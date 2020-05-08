from __future__ import absolute_import, print_function

import codecs
import io

import marshal
import types

import six

"""
A module to load python file, but also embedding namespace package logic (PEP420), implemented via pkg_resources
"""

# We need to be extra careful with python versions
# Ref : https://docs.python.org/2/library/modules.html?highlight=imports
# Ref : https://docs.python.org/3/library/modules.html?highlight=imports
import os
import sys

from ._utils import _ImportError, _verbose_message



try:
    from importlib.util import MAGIC_NUMBER
except:
    import imp
    MAGIC_NUMBER = imp.get_magic()


# This section needs to be at the beginning of the module (since spec_utils rely on it)
# It refers to Loader that will be defined later on.


# from importlib
SOURCE_SUFFIXES_2 = ['.py']
BYTECODE_SUFFIXES_2 = ['.pyc']
EXTENSION_SUFFIXES_2 = []


# Needed by _spec_utils so we load it before importing
def get_supported_file_loaders_2(force=False):
    """Returns a list of file-based module loaders.
    Each item is a tuple (loader, suffixes).
    """

    if force or (2, 7) <= sys.version_info < (3, 4):  # valid until which py3 version ?

        import imp

        loaders = []
        for suffix, mode, type in imp.get_suffixes():
            if type == imp.PY_SOURCE:
                loaders.append((SourceFileLoader2, [suffix]))
            else:
                loaders.append((ImpFileLoader2, [suffix]))
        return loaders

    elif sys.version_info >= (3, 4):  # valid from which py3 version ?

        from importlib.machinery import (
            SOURCE_SUFFIXES, SourceFileLoader,
            BYTECODE_SUFFIXES, SourcelessFileLoader,
            EXTENSION_SUFFIXES, ExtensionFileLoader,
        )

        # This is already defined in importlib._bootstrap_external
        # but is not exposed.
        extensions = ExtensionFileLoader, EXTENSION_SUFFIXES
        source = SourceFileLoader, SOURCE_SUFFIXES
        bytecode = SourcelessFileLoader, BYTECODE_SUFFIXES
        return [extensions, source, bytecode]




# backporting SourceFileLoader from python3

# We need to be extra careful with python versions
# Ref : https://docs.python.org/2/library/modules.html?highlight=imports
# Ref : https://docs.python.org/3/library/modules.html?highlight=imports
import os
import sys

import warnings
from ._utils import _ImportError, _verbose_message
from ._module_utils import ModuleSpec, module_from_spec, _new_module, _init_module_attrs
from ._encoding_utils import decode_source
from ._spec_utils import spec_from_loader


class _NamespacePath(object):
    """Represents a namespace package's path.  It uses the module name
    to find its parent module, and from there it looks up the parent's
    __path__.  When this changes, the module's own path is recomputed,
    using path_finder.  For top-level modules, the parent module's path
    is sys.path."""

    def __init__(self, name, path, path_finder):
        self._name = name
        self._path = path
        self._last_parent_path = tuple(self._get_parent_path())
        self._path_finder = path_finder

    def _find_parent_path_names(self):
        """Returns a tuple of (parent-module-name, parent-path-attr-name)"""
        parent, dot, me = self._name.rpartition('.')
        if dot == '':
            # This is a top-level module. sys.path contains the parent path.
            return 'sys', 'path'
        # Not a top-level module. parent-module.__path__ contains the
        #  parent path.
        return parent, '__path__'

    def _get_parent_path(self):
        parent_module_name, path_attr_name = self._find_parent_path_names()
        return getattr(sys.modules[parent_module_name], path_attr_name)

    def _recalculate(self):
        # If the parent's path has changed, recalculate _path
        parent_path = tuple(self._get_parent_path())  # Make a copy
        if parent_path != self._last_parent_path:
            spec = self._path_finder(self._name, parent_path)
            # Note that no changes are made if a loader is returned, but we
            #  do remember the new parent path
            if spec is not None and spec.loader is None:
                if spec.submodule_search_locations:
                    self._path = spec.submodule_search_locations
            self._last_parent_path = parent_path  # Save the copy
        return self._path

    def __iter__(self):
        return iter(self._recalculate())

    def __len__(self):
        return len(self._recalculate())

    def __repr__(self):
        return '_NamespacePath({!r})'.format(self._path)

    def __contains__(self, item):
        return item in self._recalculate()

    def append(self, item):
        self._path.append(item)


class _LoaderBasics(object):

    """Base class of common code needed by both SourceLoader and
    SourcelessFileLoader."""

    def is_package(self, fullname):
        """Concrete implementation of InspectLoader.is_package by checking if
        the path returned by get_filename has a filename of '__init__.py'."""
        filename = os.path.split(self.get_filename(fullname))[1]
        filename_base = filename.rsplit('.', 1)[0]
        tail_name = fullname.rpartition('.')[2]
        return filename_base == '__init__' and tail_name != '__init__'

    def create_module(self, spec):
        """Creates the module, and also insert it into sys.modules, adding this onto py2 import logic."""
        mod = sys.modules.setdefault(spec.name, types.ModuleType(spec.name))
        # we are using setdefault to satisfy https://docs.python.org/3/reference/import.html#loaders
        return mod

    def exec_module(self, module):
        """Execute the module."""
        code = self.get_code(module.__name__)
        if code is None:
            raise ImportError('cannot load module {!r} when get_code() '
                              'returns None'.format(module.__name__))

        exec(code, module.__dict__)

    def load_module(self, fullname):
        """Load the specified module into sys.modules and return it.
        This method is for python2 only, but implemented with backported py3 methods.
        """

        if fullname in sys.modules:
            mod = sys.modules[fullname]
            self.exec_module(mod)
            # In this case we do not want to remove the module in case of error
            # Ref : https://docs.python.org/3/reference/import.html#loaders
        else:
            try:
                # Retrieving the spec to help creating module properly
                spec = spec_from_loader(fullname, self)

                # this will call create_module and also initialize the module properly (like for py3)
                mod = module_from_spec(spec)

                # as per https://docs.python.org/3/reference/import.html#loaders
                assert mod.__name__ in sys.modules

                self.exec_module(mod)
                # We don't ensure that the import-related module attributes get
                # set in the sys.modules replacement case.  Such modules are on
                # their own.
            except Exception as exc:
                # TODO : log exception !
                # as per https://docs.python.org/3/reference/import.html#loaders
                if fullname in sys.modules:
                    del sys.modules[fullname]
                raise

        return sys.modules[fullname]

    # An old working pure python2 load_module implementation.
    # Keeping it around for reference...
    # def load_module(self, name):
    #
    #     """Load a module from a file.
    #     """
    #     # Implementation inspired from pytest.rewrite and importlib
    #
    #     # If there is an existing module object named 'name' in
    #     # sys.modules, the loader must use that existing module. (Otherwise,
    #     # the reload() builtin will not work correctly.)
    #     if name in sys.modules:
    #         return sys.modules[name]
    #
    #     # I wish I could just call imp.load_compiled here, but __file__ has to
    #     # be set properly. In Python 3.2+, this all would be handled correctly
    #     # by load_compiled.
    #     mod = sys.modules.setdefault(name, imp.new_module(name))
    #     try:
    #         # Set a few properties required by PEP 302
    #         mod.__file__ = self.get_filename(name)
    #         # this will set mod.__repr__ to not builtin...
    #         mod.__loader__ = self
    #         if self.is_package(name):
    #             mod.__path__ = [self.path]
    #             mod.__package__ = name  # PEP 366
    #         else:
    #             mod.__package__ = '.'.join(name.split('.')[:-1])  # PEP 366
    #         # if we want to skip compilation - useful for debugging
    #         # source = self.get_source(name)
    #         # exec(source, mod.__dict__)
    #         self.exec_module(mod)
    #
    #     except:
    #         if name in sys.modules:
    #             del sys.modules[name]
    #         raise
    #     return sys.modules[name]


# inspired from importlib
# Note this is NOT the same as importlib._NamespaceLoader
# Original importlib._NamespaceLoader is a loader as a hack from no_spec -> namespace feature when initializing
# whereas this one is a hack from no_spec -> namespace feature when returning a loader to actually be executed
class NamespaceLoader2(_LoaderBasics):
    """
    Loader for (Implicit) Namespace Package, inspired from importlib.
    """

    def __init__(self, name, path):
        self.name = name
        self.path = path

    def create_module(self, spec):
        """Improve python2 semantics for module creation."""
        mod = super(NamespaceLoader2, self).create_module(spec)
        # Set a few properties required by PEP 302
        # mod.__file__ = [p for p in self.path]
        # this will set mod.__repr__ to not builtin... shouldnt break anything in py2...
        # CAREFUL : get_filename present implies the module has ONE location, which is not true with namespaces
        return mod

    def load_module(self, name):
        """Load a namespace module as if coming from an empty file.
        """
        _verbose_message('namespace module loaded with path {!r}', self.path)

        # Adjusting code from LoaderBasics
        if name in sys.modules:
            mod = sys.modules[name]
            self.exec_module(mod)
            # In this case we do not want to remove the module in case of error
            # Ref : https://docs.python.org/3/reference/import.html#loaders
        else:
            try:
                # Building custom spec and loading as in _LoaderBasics...
                spec = ModuleSpec(name, self, origin='namespace', is_package=True)
                spec.submodule_search_locations = self.path

                # this will call create_module and also initialize the module properly (like for py3)
                mod = module_from_spec(spec)

                # as per https://docs.python.org/3/reference/import.html#loaders
                assert mod.__name__ in sys.modules

                self.exec_module(mod)
                # We don't ensure that the import-related module attributes get
                # set in the sys.modules replacement case.  Such modules are on
                # their own.
            except:
                # as per https://docs.python.org/3/reference/import.html#loaders
                if name in sys.modules:
                    del sys.modules[name]
                raise

        return sys.modules[name]

    def is_package(self, fullname):
        return True

    def get_source(self, name):
        # Better to not rely on anyone (pkg_resources/pkgutil) for this, since it seems we can...
        return ''

    def get_code(self, fullname):
        return compile('', '<string>', 'exec', dont_inherit=True)


class SourceLoader(_LoaderBasics):

    def set_data(self, path, data):
        """Optional method which writes data (bytes) to a file path (a str).
        Implementing this method allows for the writing of bytecode files.
        """

    def get_source(self, name):
        """Concrete implementation of InspectLoader.get_source."""
        path = self.get_filename(name)
        try:
            source_bytes = self.get_data(path)
        except OSError as exc:
            e = _ImportError('source not available through get_data()',
                             name=name)
            e.__cause__ = exc
            raise e
        return decode_source(source_bytes)

    def source_to_code(self, data, path):
        """Return the code object compiled from source.
        The 'data' argument can be any object type that compile() supports.
        """
        return compile(data, path, 'exec', dont_inherit=True)

    def get_code(self, fullname):
        source = self.get_source(fullname)
        _verbose_message('compiling code for "{0}"'.format(fullname))
        try:
            code = self.source_to_code(source, self.get_filename(fullname))
            return code
        except TypeError:
            raise


class FileLoader2(object):
    """Base class of common code needed by SourceFileLoader and ImpLoader."""

    def __init__(self, fullname, path=None):
        """Cache the module name and the path to the file found by the
                finder."""
        self.name = fullname
        self.path = path

    def __eq__(self, other):
        return (self.__class__ == other.__class__ and
                self.__dict__ == other.__dict__)

    def __hash__(self):
        return hash(self.name) ^ hash(self.path)

    def get_filename(self, fullname):
        """Return the path to the source file as found by the finder."""
        return self.path

    def get_data(self, path):
        """Return the data from path as raw bytes."""
        with io.FileIO(path, 'r') as file:
            return file.read()


# inspired from importlib2
class SourceFileLoader2(FileLoader2, SourceLoader):
    """Base file loader class which implements the loader protocol methods that
    require file system usage. Also implements implicit namespace package PEP 420.
    """

    def __init__(self, fullname, path):
        """Cache the module name and the path to the file found by the
        finder.
        :param fullname the name of the module to load
        :param path to the module or the package.
        If it is a package the path must point to a directory.
        Otherwise the path to the python module is needed
        """
        super(SourceFileLoader2, self).__init__(fullname, path)

    # Not producing bytecode here... yet.

# TODO : imp loader for frozen and builtins ??


# Implementing SourcelessFileLoader, ExtensionFileLoader for python2 with imp, to avoid unnecessary complexity
class ImpFileLoader2(FileLoader2):
    """An Import Loader for python 2.7 using imp module"""

    # Not needed but we should have it for API compatibility
    def create_module(self, spec):
        # implementation from module_from_spec
        module = _new_module(spec.name)
        return module

    # Not really needed but we should have it for API compatibility
    def exec_module(self, module):
        """Execute the module using the old imp."""
        path = [os.path.dirname(module.__file__)]  # file should have been resolved before (module creation)
        file = None
        try:
            file, pathname, description = imp.find_module(module.__name__.rpartition('.')[-1], path)
            module = imp.load_module(module.__name__, file, pathname, description)
        finally:
            if file:
                file.close()

    def load_module(self, name):
        """Load a module from a file.
        """
        # Implementation inspired from pytest.rewrite and importlib

        # If there is an existing module object named 'name' in
        # sys.modules, the loader must use that existing module. (Otherwise,
        # the reload() builtin will not work correctly.)
        if name in sys.modules:
            return sys.modules[name]
        try:
            # we have already done the search, an gone through package layers
            # so we directly feed the latest module and correct path
            # to reuse the logic for choosing the proper loading behavior

            # TODO : double check maybe we do not need the loop here, already handled by finders in dir hierarchy
            # TODO : use exec_module (recent, more tested API) from here
            for name_idx, name_part in enumerate(name.split('.')):
                pkgname = ".".join(name.split('.')[:name_idx+1])
                if pkgname not in sys.modules:
                    if '.' in pkgname:
                        # parent has to be in sys.modules. make sure it is a package, else fails
                        if '__path__' in vars(sys.modules[pkgname.rpartition('.')[0]]):
                            path = sys.modules[pkgname.rpartition('.')[0]].__path__
                        else:
                            raise ImportError("{0} is not a package (no __path__ detected)".format(pkgname.rpartition('.')[0]))
                    else:  # using __file__ instead. should always be there.
                        path = os.path.dirname(sys.modules[pkgname].__file__)if pkgname in sys.modules else None
                    try:
                        file, pathname, description = imp.find_module(pkgname.rpartition('.')[-1], path)
                        sys.modules[pkgname] = imp.load_module(pkgname, file, pathname, description)
                    finally:
                        if file:
                            file.close()
        except:
            # dont pollute the interpreter environment if we dont know what we are doing
            if name in sys.modules:
                del sys.modules[name]
            raise
        return sys.modules[name]

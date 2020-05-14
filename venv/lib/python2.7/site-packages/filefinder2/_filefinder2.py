from __future__ import absolute_import, division, print_function

import contextlib

"""
A module to find python file, but also embedding namespace package logic (PEP420)
"""

# We need to be extra careful with python versions
# Ref : https://docs.python.org/dev/library/importlib.html#importlib.import_module

import os
import sys

from ._fileloader2 import (
    _ImportError,
)

from ._spec_utils import (
    ModuleSpec,
    spec_from_file_location,
    spec_from_loader
)


from ._utils import _verbose_message
from ._fileloader2 import _NamespacePath, NamespaceLoader2
import imp
import warnings


class PathFinder2(object):
    """
    MetaFinder
    """

    @classmethod
    def invalidate_caches(cls):
        """Call the invalidate_caches() method on all path entry finders
        stored in sys.path_importer_caches (where implemented)."""
        for finder in sys.path_importer_cache.values():
            if hasattr(finder, 'invalidate_caches'):
                finder.invalidate_caches()

    @classmethod
    def _path_hooks(cls, path):  # from importlib.PathFinder
        """Search sys.path_hooks for a finder for 'path'."""
        if sys.path_hooks is not None and not sys.path_hooks:
            warnings.warn('sys.path_hooks is empty', ImportWarning)
        for hook in sys.path_hooks:
            try:
                return hook(path)
            except ImportError:
                continue
        else:
            return None

    @classmethod
    def _path_importer_cache(cls, path):  # from importlib.PathFinder
        """Get the finder for the path entry from sys.path_importer_cache.
        If the path entry is not in the cache, find the appropriate finder
        and cache it. If no finder is available, store None.
        """
        if path == '':
            try:
                path = os.getcwd()
            except FileNotFoundError:
                # Don't cache the failure as the cwd can easily change to
                # a valid directory later on.
                return None
        try:
            finder = sys.path_importer_cache[path]
        except KeyError:
            finder = cls._path_hooks(path)
            sys.path_importer_cache[path] = finder

        return finder

    @classmethod
    def _legacy_get_spec(cls, fullname, finder):
        # This would be a good place for a DeprecationWarning if
        # we ended up going that route.
        if hasattr(finder, 'find_loader'):
            loader, portions = finder.find_loader(fullname)
        else:
            loader = finder.find_module(fullname)
            portions = []
        if loader is not None:
            return spec_from_loader(fullname, loader)
        spec = ModuleSpec(fullname, None)
        spec.submodule_search_locations = portions
        return spec

    @classmethod
    def _get_spec(cls, fullname, path, target=None):
        """Find the loader or namespace_path for this module/package name."""
        # If this ends up being a namespace package, namespace_path is
        #  the list of paths that will become its __path__
        namespace_path = []
        for entry in path:
            if not isinstance(entry, (str, bytes)):
                continue
            finder = cls._path_importer_cache(entry)
            if finder is not None:
                if hasattr(finder, 'find_spec'):
                    spec = finder.find_spec(fullname, target)
                else:
                    spec = cls._legacy_get_spec(fullname, finder)
                if spec is None:
                    continue
                if spec.loader is not None:
                    return spec
                portions = spec.submodule_search_locations
                if portions is None:
                    raise ImportError('spec missing loader')
                # This is possibly part of a namespace package.
                #  Remember these path entries (if any) for when we
                #  create a namespace package, and continue iterating
                #  on path.
                namespace_path.extend(portions)
        else:
            spec = ModuleSpec(fullname, None)
            spec.submodule_search_locations = namespace_path
            return spec

    @classmethod
    def find_module(cls, fullname, path=None):
        """find the module on sys.path or 'path' based on sys.path_hooks and
        sys.path_importer_cache.
        This method is for python2 only
        """
        spec = cls.find_spec(fullname, path)
        if spec is None:
            return None
        elif spec.loader is None and spec.submodule_search_locations:
            # Here we need to create a namespace loader to handle namespaces since python2 doesn't...
            return NamespaceLoader2(spec.name, spec.submodule_search_locations)
        else:
            return spec.loader

    @classmethod
    def find_spec(cls, fullname, path=None, target=None):
        """find the module on sys.path or 'path' based on sys.path_hooks and
        sys.path_importer_cache."""
        if path is None:
            path = sys.path
        spec = cls._get_spec(fullname, path, target)
        if spec is None:
            return None
        elif spec.loader is None:
            namespace_path = spec.submodule_search_locations
            if namespace_path:
                # We found at least one namespace path.  Return a
                #  spec which can create the namespace package.
                spec.origin = 'namespace'
                spec.submodule_search_locations = _NamespacePath(fullname, namespace_path, cls._get_spec)
                return spec
            else:
                return None
        else:
            return spec


class FileFinder2(object):
    """
    FileFinder to find modules and load them via Loaders for python 2.7
    """

    def __init__(self, path, *loader_details):
        """Initialize with the path to search on and a variable number of
        2-tuples containing the loader and the file suffixes the loader
        recognizes."""
        loaders = []
        for loader, suffixes in loader_details:
            loaders.extend((suffix, loader) for suffix in suffixes)
        self._loaders = loaders
        # Base (directory) path
        self.path = path or '.'
        # Note : we are not playing with cache here (too complex to get right and not worth it for obsolete python)

        # Need to disable this to matc importlib API
        # # We need to check that we will be able to find a module or package,
        # # or raise ImportError to allow other finders to be instantiated for this path.
        # # => the logic must correspond to find_module()
        # findable = False
        # for root, dirs, files in os.walk(self.path):
        #     findable = findable or any(
        #         os.path.isfile(os.path.join(os.path.join(path, d), '__init__' + suffix))
        #         for suffix, _ in self._loaders
        #         for d in dirs
        #     ) or any(
        #         f.endswith(suffix)
        #         for suffix, _ in self._loaders
        #         for f in files
        #     )
        #
        # # CAREFUL : this is different from the FileFinder design in importlib,
        # # since we need to be able to give up (raise ImportError) here and let other finders do their jobs
        # if not findable:
        #     raise _ImportError("cannot find any matching module based on extensions {0}".format(
        #         [s for s, _ in self._loaders]),
        #         path=self.path
        #     )

    def _get_spec(self, loader_class, fullname, path, smsl, target):
        loader = loader_class(fullname, path)
        return spec_from_file_location(fullname, path, loader=loader,
                                       submodule_search_locations=smsl)

    def find_spec(self, fullname, target=None):
        """Try to find a spec for the specified module.  Returns the
        matching spec, or None if not found."""
        is_namespace = False
        tail_module = fullname.rpartition('.')[2]

        base_path = os.path.join(self.path, tail_module)
        for suffix, loader_class in self._loaders:
            init_filename = '__init__' + suffix
            init_full_path = os.path.join(base_path, init_filename)
            full_path = base_path + suffix
            if os.path.isfile(init_full_path):
                return self._get_spec(loader_class, fullname, init_full_path, [base_path], target)
            if os.path.isfile(full_path):  # maybe we need more checks here (importlib filefinder checks its cache...)
                return self._get_spec(loader_class, fullname, full_path, None, target)
        else:
            # If a namespace package, return the path if we don't
            #  find a module in the next section.
            is_namespace = os.path.isdir(base_path)

        if is_namespace:
            _verbose_message('possible namespace for {}'.format(base_path))
            spec = ModuleSpec(fullname, None)
            spec.submodule_search_locations = [base_path]
            return spec
        return None

    # def find_spec(self, fullname, target=None):
    #     """ python3 latest API, to provide a py2/py3 extensible API """
    #     path = self.path
    #     tail_module = fullname.rpartition('.')[2]
    #
    #     base_path = os.path.join(path, tail_module)
    #     for suffix, loader_class in self._loaders:
    #         full_path = None  # adjusting path for package or file
    #         if os.path.isdir(base_path) and os.path.isfile(os.path.join(base_path, '__init__' + suffix)):
    #             # __init__.py path will be computed by the loader when needed
    #             loader = loader_class(fullname, base_path)
    #         elif os.path.isfile(base_path + suffix):
    #             loader = loader_class(fullname, base_path + suffix)
    #     loader = None
    #
    #     return spec_from_loader(fullname, loader)
    #
    def find_loader(self, fullname):
        """Try to find a loader for the specified module, or the namespace
        package portions. Returns (loader, list-of-portions).
        This method is deprecated.  Use find_spec() instead.
        """
        spec = self.find_spec(fullname)
        if spec is None:
            return None, []
        return spec.loader, spec.submodule_search_locations or []

    def find_module(self, fullname):
        """Try to find a loader for the specified module, or the namespace
        package portions. Returns loader.
        """

        spec = self.find_spec(fullname)
        if spec is None:
            return None

        # We need to handle the namespace case here for python2
        if spec.loader is None and len(spec.submodule_search_locations):
            spec.loader = NamespaceLoader2(spec.name, spec.submodule_search_locations)

        return spec.loader

    @classmethod
    def path_hook(cls, *loader_details):
        """A class method which returns a closure to use on sys.path_hook
        which will return an instance using the specified loaders and the path
        called on the closure.

        If the path called on the closure is not a directory, ImportError is
        raised.

        """
        def path_hook_for_FileFinder2(path):
            """Path hook for FileFinder2."""
            if not os.path.isdir(path):
                raise _ImportError('only directories are supported', path=path)
            return cls(path, *loader_details)

        return path_hook_for_FileFinder2

    def __repr__(self):
        return 'FileFinder2({!r})'.format(self.path)




from __future__ import absolute_import, division, print_function


"""
A module for spec utils
"""

# We need to be extra careful with python versions
# Ref : https://docs.python.org/2/library/modules.html?highlight=imports
# Ref : https://docs.python.org/3/library/modules.html?highlight=imports
import os
import sys


import imp
import warnings
from ._utils import _ImportError, _verbose_message
#from ._locks import _ModuleLockManager

try:
    from importlib.machinery import ModuleSpec
except ImportError:

    # Module spec, ported from python3
    # to provide a py2/py3 API
    class ModuleSpec:
        """The specification for a module, used for loading.
        A module's spec is the source for information about the module.  For
        data associated with the module, including source, use the spec's
        loader.
        `name` is the absolute name of the module.  `loader` is the loader
        to use when loading the module.  `parent` is the name of the
        package the module is in.  The parent is derived from the name.
        `is_package` determines if the module is considered a package or
        not.  On modules this is reflected by the `__path__` attribute.
        `origin` is the specific location used by the loader from which to
        load the module, if that information is available.  When filename is
        set, origin will match.
        `has_location` indicates that a spec's "origin" reflects a location.
        When this is True, `__file__` attribute of the module is set.
        `cached` is the location of the cached bytecode file, if any.  It
        corresponds to the `__cached__` attribute.
        `submodule_search_locations` is the sequence of path entries to
        search when importing submodules.  If set, is_package should be
        True--and False otherwise.
        Packages are simply modules that (may) have submodules.  If a spec
        has a non-None value in `submodule_search_locations`, the import
        system will consider modules loaded from the spec as packages.
        Only finders (see importlib.abc.MetaPathFinder and
        importlib.abc.PathEntryFinder) should modify ModuleSpec instances.
        """

        def __init__(self, name, loader, origin=None, loader_state=None,
                     is_package=None):
            self.name = name
            self.loader = loader
            self.origin = origin
            self.loader_state = loader_state
            self.submodule_search_locations = [] if is_package else None

            # file-location attributes
            self._set_fileattr = False
            self._cached = None

        def __repr__(self):
            args = ['name={!r}'.format(self.name),
                    'loader={!r}'.format(self.loader)]
            if self.origin is not None:
                args.append('origin={!r}'.format(self.origin))
            if self.submodule_search_locations is not None:
                args.append('submodule_search_locations={}'
                            .format(self.submodule_search_locations))
            return '{}({})'.format(self.__class__.__name__, ', '.join(args))

        def __eq__(self, other):
            smsl = self.submodule_search_locations
            try:
                return (self.name == other.name and
                        self.loader == other.loader and
                        self.origin == other.origin and
                        smsl == other.submodule_search_locations and
                        #self.cached == other.cached and
                        self.has_location == other.has_location)
            except AttributeError:
                return False

        # @property
        # def cached(self):
        #     if self._cached is None:
        #         if self.origin is not None and self._set_fileattr:
        #             if _bootstrap_external is None:
        #                 raise NotImplementedError
        #             self._cached = _bootstrap_external._get_cached(self.origin)
        #     return self._cached

        # @cached.setter
        # def cached(self, cached):
        #     self._cached = cached

        @property
        def parent(self):
            """The name of the module's parent."""
            if self.submodule_search_locations is None:
                return self.name.rpartition('.')[0]
            else:
                return self.name

        @property
        def has_location(self):
            return self._set_fileattr

        @has_location.setter
        def has_location(self, value):
            self._set_fileattr = bool(value)



# def _module_repr_from_spec(spec):
#     """Return the repr to use for the module."""
#     # We mostly replicate _module_repr() using the spec attributes.
#     name = '?' if spec.name is None else spec.name
#     if spec.origin is None:
#         if spec.loader is None:
#             return '<module {!r}>'.format(name)
#         else:
#             return '<module {!r} ({!r})>'.format(name, spec.loader)
#     else:
#         if spec.has_location:
#             return '<module {!r} from {!r}>'.format(name, spec.origin)
#         else:
#             return '<module {!r} ({})>'.format(spec.name, spec.origin)




try:
    from importlib.machinery import module_from_spec
except ImportError:

    def _new_module(name):
        return type(sys)(name)

    def _init_module_attrs(spec, module, override=False):
        # The passed-in module may be not support attribute assignment,
        # in which case we simply don't set the attributes.
        # __name__
        if (override or getattr(module, '__name__', None) is None):
            try:
                module.__name__ = spec.name
            except AttributeError:
                pass
        # __loader__
        if override or getattr(module, '__loader__', None) is None:
            loader = spec.loader
            # if loader is None:
            #     # A backward compatibility hack.
            #     if spec.submodule_search_locations is not None:
            #         loader = _NamespaceLoader.__new__(_NamespaceLoader)
            #         loader.path = spec.submodule_search_locations
            try:
                module.__loader__ = loader
            except AttributeError:
                pass
        # __package__
        if override or getattr(module, '__package__', None) is None:
            try:
                module.__package__ = spec.parent
            except AttributeError:
                pass
        # __spec__
        try:
            module.__spec__ = spec
        except AttributeError:
            pass
        # __path__
        if override or getattr(module, '__path__', None) is None:
            if spec.submodule_search_locations is not None:
                try:
                    module.__path__ = spec.submodule_search_locations
                except AttributeError:
                    pass
        # __file__/__cached__
        if spec.has_location:
            if override or getattr(module, '__file__', None) is None:
                try:
                    module.__file__ = spec.origin
                except AttributeError:
                    pass
            # No cache implemented in filefinder2 currently
            # if override or getattr(module, '__cached__', None) is None:
            #     if spec.cached is not None:
            #         try:
            #             module.__cached__ = spec.cached
            #         except AttributeError:
            #             pass
        return module

    def module_from_spec(spec):
        """Create a module based on the provided spec."""
        # Typically loaders will not implement create_module().
        module = None
        if hasattr(spec.loader, 'create_module'):
            # If create_module() returns `None` then it means default
            # module creation should be used.
            module = spec.loader.create_module(spec)
        elif hasattr(spec.loader, 'exec_module'):
            warnings.warn('starting in Python 3.6, loaders defining exec_module() '
                          'must also define create_module()',
                          DeprecationWarning, stacklevel=2)
        if module is None:
            module = _new_module(spec.name)
        _init_module_attrs(spec, module)
        return module



# def _exec(spec, module):
#     """Execute the spec in an existing module's namespace."""
#     name = spec.name
#     imp.acquire_lock()
#     with _ModuleLockManager(name):
#         if sys.modules.get(name) is not module:
#             msg = 'module {!r} not in sys.modules'.format(name)
#             raise _ImportError(msg, name=name)
#         if spec.loader is None:
#             if spec.submodule_search_locations is None:
#                 raise _ImportError('missing loader', name=spec.name)
#             # namespace package
#             _init_module_attrs(spec, module, override=True)
#             return module
#         _init_module_attrs(spec, module, override=True)
#         if not hasattr(spec.loader, 'exec_module'):
#             # (issue19713) Once BuiltinImporter and ExtensionFileLoader
#             # have exec_module() implemented, we can add a deprecation
#             # warning here.
#             spec.loader.load_module(name)
#         else:
#             spec.loader.exec_module(module)
#     return sys.modules[name]


# def _load_backward_compatible(spec):
#     # (issue19713) Once BuiltinImporter and ExtensionFileLoader
#     # have exec_module() implemented, we can add a deprecation
#     # warning here.
#     spec.loader.load_module(spec.name)
#     # The module must be in sys.modules at this point!
#     module = sys.modules[spec.name]
#     if getattr(module, '__loader__', None) is None:
#         try:
#             module.__loader__ = spec.loader
#         except AttributeError:
#             pass
#     if getattr(module, '__package__', None) is None:
#         try:
#             # Since module.__path__ may not line up with
#             # spec.submodule_search_paths, we can't necessarily rely
#             # on spec.parent here.
#             module.__package__ = module.__name__
#             if not hasattr(module, '__path__'):
#                 module.__package__ = spec.name.rpartition('.')[0]
#         except AttributeError:
#             pass
#     if getattr(module, '__spec__', None) is None:
#         try:
#             module.__spec__ = spec
#         except AttributeError:
#             pass
#     return module
#
#
# class _installed_safely:
#
#     def __init__(self, module):
#         self._module = module
#         self._spec = module.__spec__
#
#     def __enter__(self):
#         # This must be done before putting the module in sys.modules
#         # (otherwise an optimization shortcut in import.c becomes
#         # wrong)
#         self._spec._initializing = True
#         sys.modules[self._spec.name] = self._module
#
#     def __exit__(self, *args):
#         try:
#             spec = self._spec
#             if any(arg is not None for arg in args):
#                 try:
#                     del sys.modules[spec.name]
#                 except KeyError:
#                     pass
#             else:
#                 _verbose_message('import {!r} # {!r}', spec.name, spec.loader)
#         finally:
#             self._spec._initializing = False
#
# def _load_unlocked(spec):
#     # A helper for direct use by the import system.
#     if spec.loader is not None:
#         # not a namespace package
#         if not hasattr(spec.loader, 'exec_module'):
#             return _load_backward_compatible(spec)
#
#     module = module_from_spec(spec)
#     with _installed_safely(module):
#         if spec.loader is None:
#             if spec.submodule_search_locations is None:
#                 raise _ImportError('missing loader', name=spec.name)
#                 # A namespace package so do nothing.
#         else:
#             spec.loader.exec_module(module)
#
#     # We don't ensure that the import-related module attributes get
#     # set in the sys.modules replacement case.  Such modules are on
#     # their own.
#     return sys.modules[spec.name]


# # A method used during testing of _load_unlocked() and by
# # _load_module_shim().
# def _load(spec):
#     """Return a new module object, loaded by the spec's loader.
#     The module is not added to its parent.
#     If a module is already in sys.modules, that existing module gets
#     clobbered.
#     """
#     imp.acquire_lock()
#     with _ModuleLockManager(spec.name):
#         return _load_unlocked(spec)



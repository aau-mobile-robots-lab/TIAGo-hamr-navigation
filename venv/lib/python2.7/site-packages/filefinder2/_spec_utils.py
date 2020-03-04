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


from ._module_utils import ModuleSpec
from ._fileloader2 import get_supported_file_loaders_2


try:
    from importlib.util import _find_spec
except ImportError:

    def _find_spec_legacy(finder, name, path):
        # This would be a good place for a DeprecationWarning if
        # we ended up going that route.
        loader = finder.find_module(name, path)
        if loader is None:
            return None
        return spec_from_loader(name, loader)

    class _ImportLockContext:

        """Context manager for the import lock."""

        def __enter__(self):
            """Acquire the import lock."""
            imp.acquire_lock()

        def __exit__(self, exc_type, exc_value, exc_traceback):
            """Release the import lock regardless of any raised exceptions."""
            imp.release_lock()

    def _find_spec(name, path, target=None):
        """Find a module's loader."""
        if sys.meta_path is not None and not sys.meta_path:
            warnings.warn('sys.meta_path is empty', ImportWarning)
        # We check sys.modules here for the reload case.  While a passed-in
        # target will usually indicate a reload there is no guarantee, whereas
        # sys.modules provides one.
        is_reload = name in sys.modules
        for finder in sys.meta_path:
            with _ImportLockContext():
                try:
                    find_spec = finder.find_spec
                except AttributeError:
                    spec = _find_spec_legacy(finder, name, path)
                    if spec is None:
                        continue
                else:
                    spec = find_spec(name, path, target)
            if spec is not None:
                # The parent import may have already imported this module.
                if not is_reload and name in sys.modules:
                    module = sys.modules[name]
                    try:
                        __spec__ = module.__spec__
                    except AttributeError:
                        # We use the found spec since that is the one that
                        # we would have used if the parent module hadn't
                        # beaten us to the punch.
                        return spec
                    else:
                        if __spec__ is None:
                            return spec
                        else:
                            return __spec__
                else:
                    return spec
        else:
            return None

try:
    from importlib.util import spec_from_file_location
except ImportError:

    # Implementing spec_from_file_location, ported from python3
    # to provide a py2/py3 API
    def spec_from_file_location(name, location=None, loader=None, submodule_search_locations=None):
        """Return a module spec based on a file location.
        To indicate that the module is a package, set
        submodule_search_locations to a list of directory paths.  An
        empty list is sufficient, though its not otherwise useful to the
        import system.
        """
        if location is None:
            # The caller may simply want a partially populated location-
            # oriented spec.  So we set the location to a bogus value and
            # fill in as much as we can.
            location = '<unknown>'
            if hasattr(loader, 'get_filename'):
                # ExecutionLoader
                try:
                    location = loader.get_filename(name)
                except ImportError:
                    pass

        # If the location is on the filesystem, but doesn't actually exist,
        # we could return None here, indicating that the location is not
        # valid.  However, we don't have a good way of testing since an
        # indirect location (e.g. a zip file or URL) will look like a
        # non-existent file relative to the filesystem.

        spec = ModuleSpec(name, loader, origin=location)
        spec._set_fileattr = True

        # Pick a loader if one wasn't provided.
        if loader is None:
            for loader_class, suffixes in get_supported_file_loaders_2():
                if location.endswith(tuple(suffixes)):
                    loader = loader_class(name, location)
                    spec.loader = loader
                    break
            else:
                return None

        # Set submodule_search_paths appropriately.
        if submodule_search_locations is None:
            # Check the loader.
            if hasattr(loader, 'is_package'):
                try:
                    is_package = loader.is_package(name)
                except ImportError:
                    pass
                else:
                    if is_package:
                        spec.submodule_search_locations = []
        else:
            spec.submodule_search_locations = submodule_search_locations
        if spec.submodule_search_locations == []:
            if location:
                dirname = os.path.split(location)[0]
                spec.submodule_search_locations.append(dirname)

        return spec


try:
    from importlib.util import spec_from_loader
except ImportError:

    # Implementing spec_from_loader, ported from python3
    # to provide a py2/py3 API
    def spec_from_loader(name, loader, origin=None, is_package=None):
        """Return a module spec based on various loader methods."""
        if hasattr(loader, 'get_filename'):
            if is_package is None:
                return spec_from_file_location(name, loader=loader)
            search = [] if is_package else None
            return spec_from_file_location(name, loader=loader,
                                           submodule_search_locations=search)

        if is_package is None:
            if hasattr(loader, 'is_package'):
                try:
                    is_package = loader.is_package(name)
                except ImportError:
                    is_package = None  # aka, undefined
            else:
                # the default
                is_package = False
        return ModuleSpec(name, loader, origin=origin, is_package=is_package)


# # Typically used by loader classes as a method replacement.
# def _load_module_shim(self, fullname):
#     """Load the specified module into sys.modules and return it.
#     This method is deprecated.  Use loader.exec_module instead.
#     """
#     spec = spec_from_loader(fullname, self)
#     if fullname in sys.modules:
#         module = sys.modules[fullname]
#         _exec(spec, module)
#         return sys.modules[fullname]
#     else:
#         return _load(spec)
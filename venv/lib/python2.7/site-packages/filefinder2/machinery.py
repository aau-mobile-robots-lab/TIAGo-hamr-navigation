from __future__ import absolute_import, print_function

import sys

# Simple module replicating importlib.machinery API of importlib in python3

from ._fileloader2 import ModuleSpec

# BuiltinImporter Not Implemented
# FrozenImporter Not implemented
# WindowsRegistryFinder
try:
    from importlib.machinery import (
        SOURCE_SUFFIXES, BYTECODE_SUFFIXES, EXTENSION_SUFFIXES
    )
except ImportError:
    from ._fileloader2 import (
        SOURCE_SUFFIXES_2, BYTECODE_SUFFIXES_2, EXTENSION_SUFFIXES_2
        # Note some of these will be different than a full fledged python import implementation.
    )
    SOURCE_SUFFIXES = SOURCE_SUFFIXES_2
    BYTECODE_SUFFIXES = BYTECODE_SUFFIXES_2
    EXTENSION_SUFFIXES = EXTENSION_SUFFIXES_2


# Should manage multiple python version by itself
def get_supported_file_loaders():
    from ._fileloader2 import get_supported_file_loaders_2
    return get_supported_file_loaders_2()


def all_suffixes():
    """Returns a list of all recognized module suffixes for this process"""
    return SOURCE_SUFFIXES + BYTECODE_SUFFIXES + EXTENSION_SUFFIXES


try:
    # Trying to import all at once (since the class hierarchy is similar)
    # I am not aware of any python implementation where we have one but not the two others...
    from importlib.machinery import SourceFileLoader, SourcelessFileLoader, ExtensionFileLoader
except ImportError:

    from ._fileloader2 import SourceFileLoader2
    from ._fileloader2 import ImpFileLoader2
    # to be compatible with py3 importlib
    SourceFileLoader = SourceFileLoader2
    SourcelessFileLoader = ImpFileLoader2
    ExtensionFileLoader = ImpFileLoader2


# Because we need to set our classes at import time
# hint : have a look at the "enforce" subpkg if you want to use
# the wrapping classes even in python3
try:
    from importlib.machinery import PathFinder as lib_pf

    PathFinder = lib_pf
except ImportError:

    from ._filefinder2 import PathFinder2
    PathFinder = PathFinder2

try:
    from importlib.machinery import FileFinder as lib_ff

    FileFinder = lib_ff
    # at import time we find the instantiated filefinder hook (because we know the index)
    try:  # DANGER : valid on python3 only ( and if imports haven't been modified previously )
        ff_path_hook = sys.path_hooks[1]
    except IndexError:
        ff_path_hook = None
except ImportError:

    from ._filefinder2 import FileFinder2
    FileFinder = FileFinder2
    ff_path_hook = FileFinder2.path_hook(*get_supported_file_loaders())


# def get_pathfinder_index_in_meta_hooks():
#         return sys.meta_path.index(PathFinder)
#
#
# def get_filefinder_index_in_path_hooks():
#     # Note the python version distinction is made at import time on ff_path_hook
#     if ff_path_hook is None:  # if it was not detected at first (pypy case)
#         # then the index is the last one, ie the length
#         idx = len(sys.path_hooks)
#     else:
#         try:
#             idx = sys.path_hooks.index(ff_path_hook)
#         except ValueError:  # if not in list it means filefinder2 was not activated.
#             # we should return the index of the original python filefinder or raise (we dont want to risk breaking imports)
#             idx = sys.path_hooks.index(ff_path_hook_original)
#
#     return idx






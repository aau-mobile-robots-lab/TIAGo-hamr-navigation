from __future__ import absolute_import, division, print_function

"""
A module for utils
"""

# We need to be extra careful with python versions
# Ref : https://docs.python.org/2/library/modules.html?highlight=imports
# Ref : https://docs.python.org/3/library/modules.html?highlight=imports
import os
import sys


def _verbose_message(message, *args, **kwargs):
    """Print the message to stderr if -v/PYTHONVERBOSE is turned on."""
    verbosity = kwargs.pop('verbosity', 1)
    if sys.flags.verbose >= verbosity:
        if not message.startswith(('#', 'import ')):
            message = '# ' + message
        print(message.format(*args), file=sys.stderr)


# From importlib, just checking if ImportError accept name and path
try:
    ImportError('msg', name='name', path='path')
except TypeError:
    class _ImportError(ImportError):
        """Implementing Import Error with name and path args"""
        def __init__(self, *args, **kwargs):
            self.name = kwargs.pop('name', None)
            self.path = kwargs.pop('path', None)
            super(_ImportError, self).__init__(*args, **kwargs)
else:
    _ImportError = ImportError

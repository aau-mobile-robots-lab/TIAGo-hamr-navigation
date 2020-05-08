from __future__ import absolute_import, division, print_function

import contextlib
import importlib
import site
import tempfile

import shutil


from rosimport import genrosmsg_py, genrossrv_py

"""
A module to setup custom importer for .msg and .srv files
Upon import, it will first find the .msg file, then generate the python module for it, then load it.

TODO...
"""

# We need to be extra careful with python versions
# Ref : https://docs.python.org/dev/library/importlib.html#importlib.import_module

# Ref : http://stackoverflow.com/questions/67631/how-to-import-a-module-given-the-full-path
# Note : Couldn't find a way to make imp.load_source deal with packages or relative imports (necessary for our generated message classes)
import os
import sys
import logging


# Class to allow dynamic search of packages
class RosSearchPath(dict):
    """
    Class to allow dynamic search of packages.
    This is where we hook up into python import mechanism in order to generate and discover
    packages and messages we are depending on.

    But it should not be used during the generation of multiple messages in only one package,
    as this is too tricky to get right, and too easy to break by mistake.
    """
    def __init__(self, **ros_package_paths):
        # we use the default ROS_PACKAGE_PATH if already setup in environment.
        # This allows us to find message definitions in a ROS distro (and collaborate with pyros_setup)
        package_paths = {}
        for distropath in [d for d in os.environ.get('ROS_PACKAGE_PATH', '').split(':') if os.path.exists(d)]:
            for p in [pkgd for pkgd in os.listdir(distropath) if os.path.exists(os.path.join(distropath, pkgd, 'msg'))]:
                package_paths[p] = package_paths.get(p, set()) | {os.path.join(distropath, p, 'msg')}

        # we add any extra path
        package_paths.update(ros_package_paths)
        super(RosSearchPath, self).__init__(package_paths)

    def try_import(self, item):
        try:
            # we need to import the .msg submodule (only one usable as dependency)
            mod = importlib.import_module(item + '.msg')
            # import succeeded : we should get the namespace path
            # and add it to the list of paths to avoid going through this all over again...
            for p in mod.__path__:
                # Note we want dependencies here. dependencies are ALWAYS '.msg' files in 'msg' directory.
                msg_path = os.path.join(p)
                # We add a path only if we can find the 'msg' directory
                self[item] = self.get(item, set() | ({msg_path} if os.path.exists(msg_path) else set()))
            return mod
        except ImportError:
            # import failed
            return None

    def __contains__(self, item):
        """ True if D has a key k, else False. """
        has = super(RosSearchPath, self).__contains__(item)
        if not has:  # attempt importing. solving ROS path setup problem with python import paths setup.
            self.try_import(item)
            # Note : if ROS is setup, rospkg.RosPack can find packages
        # try again (might work now)
        return super(RosSearchPath, self).__contains__(item)

    def __getitem__(self, item):
        """ x.__getitem__(y) <==> x[y] """
        got = super(RosSearchPath, self).get(item)
        if got is None:
            # attempt discovery by relying on python core import feature.
            self.try_import(item)
            # Note : if ROS is setup, rospkg.RosPack can find packages
        return super(RosSearchPath, self).get(item)

# singleton instance, to keep used ros package paths in cache
ros_import_search_path = RosSearchPath()


def RosLoader(rosdef_extension):
    """
    Function generating ROS loaders.
    This is used to keep .msg and .srv loaders very similar
    """
    if rosdef_extension == '.msg':
        loader_origin_subdir = 'msg'
        loader_file_extension = rosdef_extension
        loader_generated_subdir = 'msg'
        loader_generator = genrosmsg_py
    elif rosdef_extension == '.srv':
        loader_origin_subdir = 'srv'
        loader_file_extension = rosdef_extension
        loader_generated_subdir = 'srv'
        loader_generator = genrossrv_py
    else:
        raise RuntimeError("RosLoader for a format {0} other than .msg or .srv is not supported".format(rosdef_extension))

    import filefinder2.machinery

    class ROSDefLoader(filefinder2.machinery.SourceFileLoader):
        """
        Python Loader for Rosdef files.
        Note : We support ROS layout :
        - msg/myMsg.msg
        - srv/mySrv.srv
        - my_pkg/__init__.py  # doesnt really matters ( we rely on PEP 420 )
        OR inside the python code:
        - my_pkg/__init__.py  # doesnt really matters ( we rely on PEP 420 )
        - my_pkg/msg/myMsg.msg
        - my_pkg/srv/mySrv.srv

        BUT the following is also importable relatively,
        which is especially useful for tests or intra-package ROS communication,
        although it cannot be used as another package dependency (due to ROS limitations)

        - my_pkg/__init__.py  # doesnt really matters ( we rely on PEP 420 )
        - my_pkg/subpkg/__init__.py # doesnt really matters ( we rely on PEP 420 )
        - my_pkg/subpkg/msg/myMsg.msg
        - my_pkg/subpkg/srv/mySrv.srv

        In that case myMsg.py will also be generated under mypkg.msg,
        but can be imported relatively from my_pkg/subpkg/module.py with "from .msg import mypkg"
        """

        rosimport_tempdir = os.path.join(tempfile.gettempdir(), 'rosimport')

        def __init__(self, fullname, path):

            self.logger = logging.getLogger(__name__)
            # to normalize input
            path = os.path.normpath(path)

            # Doing this in each loader, in case we are running from different processes,
            # avoiding to reload from same file (especially useful for boxed tests).
            # But deterministic path to avoid regenerating from the same interpreter
            rosimport_path = os.path.join(self.rosimport_tempdir, str(os.getpid()))
            if not os.path.exists(rosimport_path):
                os.makedirs(rosimport_path)

            rospackage = fullname.partition('.')[0]

            if os.path.isdir(path):
                # if we get a package name ending with msg or srv and a non empty directory
                if (
                            fullname.endswith(loader_origin_subdir) and
                            any([f.endswith(loader_file_extension) for f in os.listdir(path)])
                ):

                    # TODO : dynamic in memory generation (we do not need the file ultimately...)
                    outdir, gen_rosdef_pkgpath = loader_generator(
                        # generate message's python code at once, for this package level.
                        rosdef_files=[os.path.join(path, f) for f in os.listdir(path)],
                        package=fullname,
                        sitedir=rosimport_path,
                        search_path=ros_import_search_path,
                    )
                    # TODO : handle thrown exception (cleaner than hacking the search path dict...)
                    # try:
                    #     generator.generate_messages(package, rosfiles, outdir, search_path)
                    # except genmsg.MsgNotFound as mnf:
                    #     try:
                    #         mod = importlib.import_module(mnf.package)
                    #         # import succeeded : we should get the namespace path that has '/msg'
                    #         # and add it to the list of paths to avoid going through this all over again...
                    #         for p in mod.__path__:
                    #             # Note we want dependencies here. dependencies are ALWAYS '.msg' files in 'msg' directory.
                    #             msg_path = os.path.join(p, genmsg_MSG_DIR)
                    #             # We add a path only if we can find the 'msg' directory
                    #             search_path[mnf.package] = search_path[mnf.package] + ([msg_path] if os.path.exists(msg_path) else [])
                    #         # Try generation again
                    #         generator.generate_messages(package, rosfiles, outdir, search_path)
                    #     except ImportError:
                    #         # import failed
                    #         return None

                    if not os.path.exists(gen_rosdef_pkgpath):
                        raise ImportError("{0} file not found".format(gen_rosdef_pkgpath))

                    # relying on usual source file loader since we have generated normal python code
                    super(ROSDefLoader, self).__init__(fullname, gen_rosdef_pkgpath)

        def get_gen_path(self):
            """Returning the generated path matching the import"""
            return self.path  # TODO : maybe useless ?
            # return os.path.join(self.outdir_pkg, loader_generated_subdir)

        def __repr__(self):
            return "ROSDefLoader/{0}({1}, {2})".format(loader_file_extension, self.name, self.path)

        @staticmethod
        def get_file_extension():
            return loader_file_extension

        @staticmethod
        def get_origin_subdir():
            return loader_origin_subdir

        @staticmethod
        def get_generated_subdir():
            return loader_generated_subdir

    return ROSDefLoader

ROSMsgLoader = RosLoader(rosdef_extension='.msg')
ROSSrvLoader = RosLoader(rosdef_extension='.srv')

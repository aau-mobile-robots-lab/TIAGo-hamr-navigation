from __future__ import absolute_import, division, print_function

import contextlib
import importlib
import site

from rosimport import rosmsg_generator

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

# This will take the ROS distro version if ros has been setup
import genpy.generator
import genpy.generate_initpy

import logging





if sys.version_info >= (3, 4):

    import importlib.abc
    import importlib.machinery

    class ROSDirectoryMetaFinder(importlib.machinery.PathFinder):

        @classmethod
        def find_module(cls, fullname, path=None):  # from importlib.PathFinder
            pass

    class ROSDistroMetaFinder(importlib.machinery.PathFinder):
        def __init__(self, *workspaces):
            """
            :param workspaces: can be a devel or install workspace (including a distro install directory), but also a directory containing packages (like a source workspace)
            These should all work, without catkin build necessary.
            """
            super(ROSDistroMetaFinder, self).__init__()
            # TODO : prevent that when we are in virtualenv and we have not allowed system site packages

            self.src_rospkgs = []

            broken_workspaces = []
            for w in workspaces:
                # Adding the share path (where we can find all packages and their messages)
                share_path = os.path.join(w, 'share')
                if os.path.exists(share_path):
                    self.share_path = share_path
                else:  # this is not a workspace, maybe we are expected to get the package directly from source ?
                    found_one = False
                    for root, dirs, files in os.walk(w, topdown=False):
                        if 'package.xml' in files:  # we have found a ros package
                            self.src_rospkgs.append(root)
                            found_one = True
                    if not found_one:
                        broken_workspaces.append(w)

                    raise ImportError

                python_libpath = os.path.join(w, 'lib', 'python' + sys.version_info.major + '.' + sys.version_info.minor)
                if os.path.exists(python_libpath):
                    # adding python site directories for the ROS distro (to find python modules as usual with ROS)
                    if os.path.exists(os.path.join(python_libpath, 'dist-packages')):
                        site.addsitedir(os.path.join(python_libpath, 'dist-packages'))
                    if os.path.exists(os.path.join(python_libpath, 'site-packages')):
                        site.addsitedir(os.path.join(python_libpath, 'site-packages'))
                else:  # this is not a workspace, maybe we are expected to get the package directly from source ?

                    for root, dirs, files in os.walk(w, topdown=False):
                        if 'package.xml' in files:  # we have found a ros package
                            self.src_rospkgs.append(root)

                    raise ImportError




        def find_spec(self, fullname, path, target=None):
            """
            :param fullname: the name of the package we are trying to import
            :param path: path we we expect to find it (can be None)
            :param target: what we plan to do with it
            :return:
            """
            # TODO: read PEP 420 :)
            last_mile = fullname.split('.')[-1]

            path = path or sys.path

            # TODO: we should handle different structure of ROS packages (installed or not)
            #if (os.path.exists)

            # TODO : similar to pyros-setup ?
            # + setup rosmsg importers ?
            if (not os.path.exists(os.path.join(last_mile, 'package.xml')) or
            False):
                raise ImportError

            for p in path:
                fullpath = os.path.join(p, last_mile)
                init_fullpath = os.path.join(fullpath, '__init__.ay')
                module_fullpath = fullpath + '.ay'

                if os.path.isdir(fullpath) and os.path.exists(init_fullpath):
                    return importlib.machinery.ModuleSpec(
                        fullname,
                        loader,
                        origin=init_fullpath
                    )

                else:
                    if os.path.exists(module_fullpath):
                        return importlib.machinery.ModuleSpec(
                            fullname,
                            loader,
                            origin=module_fullpath
                        )

            return None

else:
    pass  # TODO


# Useless ?
#_ros_finder_instance_obsolete_python = ROSImportFinder

ros_distro_finder = None


def activate(rosdistro_path=None, *workspaces):
    global ros_distro_finder
    if rosdistro_path is None:  # autodetect most recent installed distro
        if os.path.exists('/opt/ros/lunar'):
            rosdistro_path = '/opt/ros/lunar'
        elif os.path.exists('/opt/ros/kinetic'):
            rosdistro_path = '/opt/ros/kinetic'
        elif os.path.exists('/opt/ros/jade'):
            rosdistro_path = '/opt/ros/jade'
        elif os.path.exists('/opt/ros/indigo'):
            rosdistro_path = '/opt/ros/indigo'
        else:
            raise ImportError(
                "No ROS distro detected on this system. Please specify the path when calling ROSImportMetaFinder()")

    ros_distro_finder = ROSImportMetaFinder(rosdistro_path, *workspaces)
    sys.meta_path.append(ros_distro_finder)

    #if sys.version_info >= (2, 7, 12):  # TODO : which exact version matters ?

    # We need to be before FileFinder to be able to find our (non .py[c]) files
    # inside, maybe already imported, python packages...
    sys.path_hooks.insert(1, ROSImportFinder)

    # else:  # older (trusty) version
    #     sys.path_hooks.append(_ros_finder_instance_obsolete_python)

    for hook in sys.path_hooks:
        print('Path hook: {}'.format(hook))

    # TODO : mix that with ROS PYTHONPATH shenanigans... to enable the finder only for 'ROS aware' paths
    if paths:
        sys.path.append(*paths)


def deactivate(*paths):
    """ CAREFUL : even if we remove our path_hooks, the created finder are still cached in sys.path_importer_cache."""
    #if sys.version_info >= (2, 7, 12):  # TODO : which exact version matters ?
    sys.path_hooks.remove(ROSImportFinder)
    # else:  # older (trusty) version
    #     sys.path_hooks.remove(_ros_finder_instance_obsolete_python)
    if paths:
        sys.path.remove(*paths)

    sys.meta_path.remove(ros_distro_finder)


@contextlib.contextmanager
def ROSImportContext(*paths):
    activate(*paths)
    yield
    deactivate(*paths)


# TODO : a meta finder could find a full ROS distro...

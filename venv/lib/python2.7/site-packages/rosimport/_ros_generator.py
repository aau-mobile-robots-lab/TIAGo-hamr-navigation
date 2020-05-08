from __future__ import absolute_import, division, print_function

import os
import sys
import tempfile
import traceback
import importlib
import site
import time

import collections
import pkg_resources

"""
Module that can be used standalone, or as part of the rosimport package
It provides a set of functions to generate your ros messages, even when ROS is not installed on your system.

This module allows to generate all rosdef message/service python code for one ros package, including subpackages.

This module does NOT provide a solution for one package to depend on another.
That problem will be handled by leveraging the python import system.

Note : Generated modules/packages can only be imported once. So it is important to provide an API that : 
- makes it easy to generate the whole module/package at once, since this is our priority
- makes it easy to optionally import the whole generated module/package
- still allows to generate only one module / a part of the whole package, caveats apply / warning added.
"""

try:
    # Using genpy and genmsg directly if ROS has been setup (while using from ROS pkg)
    import genmsg as genmsg
    import genmsg.command_line as genmsg_command_line
    import genpy.generator as genpy_generator
    import genpy.generate_initpy as genpy_generate_initpy

except ImportError:

    # Otherwise we refer to our submodules here (setup.py usecase, or running from tox without site-packages)

    import site
    ros_site_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'ros-packages')
    print("Adding site directory {ros_site_dir} to access genpy and genmsg.".format(**locals()))
    site.addsitedir(ros_site_dir)

    import genmsg as genmsg
    import genmsg.command_line as genmsg_command_line
    import genpy.generator as genpy_generator
    import genpy.generate_initpy as genpy_generate_initpy

    # Note we do not want to use pyros_setup here.
    # We do not want to do a full ROS setup, only import specific packages.
    # If needed it should have been done before (loading a parent package).
    # this handle the case where we want to be independent of any underlying ROS system.

# TMP bwcompat
try:
    import genmsg.MSG_DIR as genmsg_MSG_DIR
    import genmsg.SRV_DIR as genmsg_SRV_DIR
except ImportError:  # bwcompat
    genmsg_MSG_DIR = 'msg'
    genmsg_SRV_DIR = 'srv'


class MsgGenerationFailed(Exception):
    pass


class MsgDependencyNotFound(Exception):
    pass


class PkgAlreadyExists(Exception):
    pass


def _generator_factory(generator, directory_name, file_extension):

    def _generator_py_pkg(files, package, outdir, search_path=None, initpy=True):
        """
        Generates python code from ROS definition files
        :param files: the list of ros definition files to generate from
        :param package: the ROS package for which we are generating messages / services
        :param outdir: the directory where to output the code generated for this package.
        :param search_path: a dict where keys are ROS package names, and value is a list of path to directory containing '.msg' files
        :param initpy: Whether or not generate the __init__.py for the package
        :return:
        """

        # Note only msg files can be used as dependencies.
        # setting search path here ensures it is stored in ros_search_path
        search_path = {} if search_path is None else search_path

        # checking if we have files with unknown extension to except early
        for f in files:
            if not f.endswith(file_extension):
                print("WARNING: {f} doesnt have the proper {file_extension} extension. It has been Ignored.".format(**locals()), file=sys.stderr)

        filtered_files = [f for f in files if f.endswith(file_extension)]
        genset = set()

        if filtered_files:

            if not os.path.exists(outdir):
                # This script can be run multiple times in parallel. We
                # don't mind if the makedirs call fails because somebody
                # else snuck in and created the directory before us.
                try:
                    os.makedirs(outdir)
                except OSError as e:
                    if not os.path.exists(outdir):
                        raise
            try:
                retcode = generator.generate_messages(
                    package=package,
                    package_files=filtered_files,
                    outdir=outdir,
                    search_path=search_path
                )
                assert retcode == 0

                # because the OS interface might not be synchronous....
                while not os.path.exists(outdir):
                    time.sleep(.1)

            except genmsg.InvalidMsgSpec as e:
                print("ERROR: ", e, file=sys.stderr)
                raise
            except genmsg.MsgGenerationException as e:
                print("ERROR: ", e, file=sys.stderr)
                raise
            except Exception as e:
                traceback.print_exc()
                print("ERROR: ", e)
                raise

            # optionally we can generate __init__.py
            if initpy:
                init_path = os.path.join(outdir, '__init__.py')
                # Note if it already exists, we overwrite it. This should accumulate generated modules.
                genpy_generate_initpy.write_modules(outdir)
                genset.add(init_path)
            else:  # we list all files, only if init.py was not created (and user has to import one by one)
                for f in files:
                    f, _ = os.path.splitext(f)  # removing extension
                    os.path.relpath(outdir)
                    genset.add(os.path.join(outdir, '_' + os.path.basename(f) + '.py'))

        return genset

    return _generator_py_pkg

# TODO : get extensions and dir from genmsg
_genmsgpkg_py = _generator_factory(genpy_generator.MsgGenerator(), 'msg', '.msg')
_gensrvpkg_py = _generator_factory(genpy_generator.SrvGenerator(), 'srv', '.srv')


def genrosmsg_py(rosdef_files, package, sitedir, search_path=None):
    """
    Generates message/services modules for a package, in that package directory,
    in a subpackage called 'msg'/'srv', following ROS conventions
    :param rosdef_files: the .msg/.srv files to use as input for generating the python message classes
    :param package: the package for which we want to generate these messages.
    This is a string separated with '.', the head element being the ros package,
    the tail being the optional python subpackage where the code will be generated.
    :param sitedir: the site directory where to put the generated package.
    :param search_path: optionally a mapping of the form {package: [list of paths]} , in order to retrieve message dependencies
    Note that all dependencies must have been previously generated, or passed in the rosdef_files list,
    otherwise generation will fail if not included in search_path, or import will fail afterwards...
    :return: the list of files generated
    """

    # Computing outdir_pkg from sitedir and package
    rospackage = package.partition('.')[0]

    outdir = sitedir
    for pkg in package.split('.'):
        # removing the trailing 'msg' if present, to enforce it if not
        if not (package.endswith(pkg) and pkg == 'msg'):
            outdir = os.path.join(outdir, pkg)

    search_path = {} if search_path is None else search_path

    # modifying the search path for later retrieval of msg as dependencies,
    # if the caller keep it around...
    search_path.setdefault(rospackage, {os.path.dirname(m) for m in rosdef_files})
    # Need to be done before generation to avoid useless harmful recursive self imports

    # Using tuple unpacking to get the unique element of the set (since we ask for init.py)
    (generated_pkg,) = _genmsgpkg_py(
        files=[f for f in rosdef_files if f.endswith('.msg')],
        package=rospackage,
        outdir=os.path.join(outdir, 'msg'),
        search_path=search_path,
        initpy=True
    )

    return sitedir, generated_pkg


def genrossrv_py(rosdef_files, package, sitedir, search_path=None):
    """
    Generates message/services modules for a package, in that package directory,
    in a subpackage called 'msg'/'srv', following ROS conventions
    :param rosdef_files: the .msg/.srv files to use as input for generating the python message classes
    :param package: the package for which we want to generate these messages.
    This is a string separated with '.', the head element being the ros package,
    the tail being the optional python subpackage where the code will be generated.
    :param sitedir: the site directory where to put the generated package.
    :param search_path: optionally a mapping of the form {package: [list of paths]} , in order to retrieve message dependencies
    Note that all dependencies must have been previously generated, or passed in the rosdef_files list,
    otherwise generation will fail if not included in search_path, or import will fail afterwards...
    :return: the list of files generated
    """

    # Computing outdir_pkg from sitedir and package
    rospackage = package.partition('.')[0]

    outdir = sitedir
    for pkg in package.split('.'):
        # removing the trailing 'srv' if present, to enforce it if not
        if not (package.endswith(pkg) and pkg == 'srv'):
            outdir = os.path.join(outdir, pkg)

    search_path = search_path or {}

    # Here the include path has already been added to ros_search_path,
    # and can be used to discover msg as dependencies
    # Using tuple unpacking to get the unique element of the set (since we ask for init.py)
    (generated_pkg, ) = _gensrvpkg_py(
        files=[f for f in rosdef_files if f.endswith('.srv')],
        package=rospackage,
        outdir=os.path.join(outdir, 'srv'),
        search_path=search_path,
        initpy=True
    )

    return sitedir, generated_pkg


# def generate_rosdefs_py(files, package, sitedir=None):
#     """
#     Generates ros messages python modules for a set of rosdefs files.
#     :param files: the rosdef files to generate python code from
#     :param package: the package these rosdefs belong to.
#     This is a string separated with '.', the head element being the ros package,
#     the tail being the optional python subpackage where the code will be generated.
#     :param sitedir: the output dir where to put the generated code for the (base / ros) package
#     :return:
#     """
#     # TODO : since we return a full package, we should probably pass a dir, not the files one by one...
#     # by default we generate for this package (does it make sense ?)
#
#     sitedir = sitedir or tempfile.mkdtemp('rosimport_generated_site')
#
#     # If our sitedir is not inside an already registered path
#     parent_in_sys_path = False
#     sitedir_splitted = sitedir.split(os.sep)
#     for i, _ in enumerate(sitedir_splitted):
#         if os.path.join('/' if sitedir_splitted[0] == '' else '', *sitedir_splitted[:len(sitedir_splitted)-i]) in sys.path:
#             parent_in_sys_path = True
#             break
#     if not parent_in_sys_path:
#         # We need to add the parent of our generated directory, for any messages dependency to resolve properly
#         # and be able to import from it as usual, even while generating python code...
#         site.addsitedir(sitedir)  # we add our output dir as a site
#         # This is required to be able to access generated python code
#
#     outdir, gen_files = genros_py(
#         rosdef_files=files,
#         package=package,
#         sitedir=sitedir,
#     )
#
#     # computing module names that will be importable after outdir has been added as sitedir
#     gen_msgs = None
#     gen_srvs = None
#     # Not ideal, but it will do until we implement a custom importer
#     for f in gen_files:
#         f = f[:-len('.py')] if f.endswith('.py') else f
#         f = f[:-len(os.sep + '__init__')] if f.endswith(os.sep + '__init__') else f
#
#         if f.endswith('msg'):
#             gen_msgs = package + '.msg'
#
#         if f.endswith('srv'):
#             gen_srvs = package + '.srv'
#
#     # we need to get one level up to get the sitedir (containing the generated namespace package)
#     return sitedir, gen_msgs, gen_srvs


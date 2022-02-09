import os
import platform
import subprocess
import sys
from setuptools import setup, find_packages, Extension
from os.path import join, dirname
from setuptools.command.build_ext import build_ext
from pprint import pprint

# Filename for the C extension module library
c_module_name = '_native'

# Command line flags forwarded to CMake (for debug purpose)
cmake_cmd_args = []
for f in sys.argv:
    if f.startswith('-D'):
        cmake_cmd_args.append(f)

for f in cmake_cmd_args:
    sys.argv.remove(f)


def _get_env_variable(name, default='OFF'):
    if name not in os.environ.keys():
        return default
    return os.environ[name]


class CMakeExtension(Extension):
    def __init__(self, name, cmake_lists_dir='.', sources=[], **kwa):
        Extension.__init__(self, name, sources=sources, **kwa)
        self.cmake_lists_dir = os.path.abspath(cmake_lists_dir)


class CMakeBuild(build_ext):
    def build_extensions(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError('Cannot find CMake executable')

        for ext in self.extensions:

            extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
            cfg = 'Debug' if _get_env_variable('DISPTOOLS_DEBUG') == 'ON' else 'Release'

            cmake_args = [
                '-DCMAKE_BUILD_TYPE=%s' % cfg,
                '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir),
                '-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), self.build_temp),
                '-DDJINNI_WITH_PYTHON=ON',
                '-DDJINNI_WITH_OBJC=OFF',
                '-DDJINNI_WITH_JNI=OFF',
                '-DDJINNI_BUILD_TESTING=OFF',
                '-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON'
            ]

            if platform.system() == 'Windows':
                plat = ('x64' if platform.architecture()[0] == '64bit' else 'Win32')
                cmake_args += [
                    '-DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE',
                    '-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir),
                ]
                if self.compiler.compiler_type == 'msvc':
                    cmake_args += [
                        '-DCMAKE_GENERATOR_PLATFORM=%s' % plat,
                    ]
                else:
                    cmake_args += [
                        '-G', 'MinGW Makefiles',
                    ]

            cmake_args += cmake_cmd_args

            pprint(cmake_args)

            if not os.path.exists(self.build_temp):
                os.makedirs(self.build_temp)

            # Config and build the extension
            subprocess.check_call(['cmake', ext.cmake_lists_dir] + cmake_args,
                                  cwd=self.build_temp)
            subprocess.check_call(['cmake', '--build', '.', '--config', cfg, '--', '-j'],
                                  cwd=self.build_temp)


all_packages = []
for pkg in find_packages():
    if "demos" in pkg:
        pkg = "ocelot." + pkg
    all_packages.append(pkg)

all_packages.append('djinni')
all_packages.append('ocelot.djinni');

setup(
    name='ocelot',
    version='20.11.2',
    description='Accelerator, radiation and x-ray optics simulation framework',
    author='ocelot-collab',
    author_email='tomin.sergey@gmail.com',
    url='https://github.com/ocelot-collab/ocelot',
    packages=all_packages,
    package_dir={
        'ocelot.demos': 'demos',
         'djinni' : 'djinni-support-lib/djinni/py/djinni',
         'ocelot.djinni' : 'generated/ocelot/djinni/py'
         },  ## install examples along with the rest of the source
    install_requires=[
        'numpy', 'scipy', 'matplotlib', 'pandas', 'cffi', 'future'
    ],
    extras_require={'docs': ['Sphinx', 'alabaster', 'sphinxcontrib-jsmath']},
    ext_modules=[CMakeExtension(c_module_name)],
    cmdclass={'build_ext': CMakeBuild},
    package_data={'ocelot.optics': ['data/*.dat']},
    license="GNU General Public License v3.0",
)

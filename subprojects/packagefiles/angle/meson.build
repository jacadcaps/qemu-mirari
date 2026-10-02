# SPDX-License-Identifier: GPL-2.0-or-later
#
# meson glue for the vendored ANGLE checkout.
#
# ANGLE is built with Chromium's GN/gclient toolchain rather than meson, so the
# real work happens in angle-build.py.  libepoxy - the only consumer here -
# needs ANGLE's headers at build time and dlopen()s libEGL.dylib/libGLESv2.dylib
# at run time, so the dependencies exported below are headers-only and the
# dylibs are produced as an installed target instead of something to link.

project('angle', 'c', version: '2.1.0', license: 'BSD-3-Clause')

python = import('python').find_installation()

angle_libs = custom_target(
  'angle-libs',
  output: ['libEGL.dylib', 'libGLESv2.dylib'],
  command: [
    python, files('angle-build.py'),
    '--source-dir', meson.current_source_dir(),
    '--build-dir', meson.current_build_dir(),
    '--cpu', host_machine.cpu_family() == 'aarch64' ? 'arm64' : 'x64',
    '@OUTPUT0@', '@OUTPUT1@',
  ],
  build_by_default: true,
  install: true,
  install_dir: get_option('libdir'),
)

angle_egl_dep = declare_dependency(
  include_directories: include_directories('include'),
  version: meson.project_version(),
)
angle_glesv2_dep = angle_egl_dep

meson.override_dependency('egl', angle_egl_dep)
meson.override_dependency('glesv2', angle_glesv2_dep)

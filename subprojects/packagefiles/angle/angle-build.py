#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Build ANGLE's libEGL/libGLESv2 from the vendored checkout.

ANGLE uses Chromium's GN/gclient tooling rather than meson, so this script
drives that toolchain from a meson custom_target.  It is deliberately
idempotent: the expensive steps (depot_tools clone, ``gclient sync``) are
skipped when their outputs are already in place, so incremental rebuilds are
cheap.
"""

import argparse
import os
import shutil
import subprocess
import sys

DEPOT_TOOLS_URL = "https://chromium.googlesource.com/chromium/tools/depot_tools"
# Pinned so that a rebuild months from now reproduces today's toolchain.
DEPOT_TOOLS_REV = "81577f19a8497ba7e41afac322e8f03553a863ec"

GN_ARGS = [
    "is_debug=false",
    # Link against the system libc++; ANGLE's bundled copy conflicts with the
    # one QEMU and its other dependencies already use.
    "use_custom_libcxx=false",
    # ANGLE is warning-clean only against the Clang it pins, not the Xcode one.
    "treat_warnings_as_errors=false",
    "angle_build_tests=false",
    "angle_enable_vulkan=false",
    # Link with Apple's linker rather than the pinned lld.  The macOS 27 SDK
    # tags its .tbd stubs with an architecture (arm64e.x1) the bundled lld's
    # TAPI reader does not know, and it rejects the whole file rather than
    # skipping the unknown slice, so every system library fails to load.
    # Chromium supports the Apple linker on macOS; ANGLE does not need lld.
    "use_lld=false",
]


def check_metal_toolchain():
    """Fail early, and legibly, if the Metal compiler is only a stub.

    Xcode 26 moved the Metal toolchain into a separately downloaded
    component.  Without it ANGLE's Metal backend fails deep inside a ninja
    run with a message that is easy to miss.
    """
    probe = subprocess.run(["xcrun", "metal", "-x", "metal", "-c", "/dev/null",
                            "-o", os.devnull],
                           capture_output=True, text=True)
    if "MetalToolchain" in probe.stderr:
        sys.exit("ANGLE needs Xcode's Metal toolchain, which is not installed.\n"
                 "Install it with:\n\n"
                 "    xcodebuild -downloadComponent MetalToolchain\n")


def run(cmd, cwd, env):
    print("+ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=cwd, env=env, check=True)


def ensure_depot_tools(path):
    if not os.path.exists(os.path.join(path, "gclient")):
        subprocess.run(["git", "clone", "--filter=blob:none", DEPOT_TOOLS_URL, path],
                       check=True)
    subprocess.run(["git", "checkout", "--detach", DEPOT_TOOLS_REV], cwd=path,
                   check=True)
    return path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source-dir", required=True,
                   help="the ANGLE checkout (the meson subproject dir)")
    p.add_argument("--build-dir", required=True,
                   help="scratch directory for depot_tools and GN output")
    p.add_argument("--cpu", required=True, help="GN target_cpu")
    p.add_argument("outputs", nargs="+",
                   help="dylibs to copy out of the GN build directory")
    args = p.parse_args()

    check_metal_toolchain()

    src = os.path.abspath(args.source_dir)
    scratch = os.path.abspath(args.build_dir)
    os.makedirs(scratch, exist_ok=True)

    depot_tools = ensure_depot_tools(os.path.join(scratch, "depot_tools"))

    env = dict(os.environ)
    env["PATH"] = depot_tools + os.pathsep + env["PATH"]
    # depot_tools self-updating mid-build would defeat the pin above.
    env["DEPOT_TOOLS_UPDATE"] = "0"
    env["DEPOT_TOOLS_METRICS"] = "0"

    # DEPOT_TOOLS_UPDATE=0 suppresses the implicit bootstrap that fetches the
    # CIPD-managed python/gn/ninja, which `gn gen` then cannot find.  Ask for it
    # explicitly instead of letting depot_tools update itself off the pin.
    run(["./ensure_bootstrap"], depot_tools, env)

    if not os.path.exists(os.path.join(src, ".gclient")):
        run([sys.executable, "scripts/bootstrap.py"], src, env)
    run(["gclient", "sync", "-D", "--no-history"], src, env)

    out = os.path.join(scratch, "gn")
    run(["gn", "gen", out, "--args=%s" % " ".join(
        GN_ARGS + ['target_cpu="%s"' % args.cpu])], src, env)
    run(["ninja", "-C", out, "libEGL", "libGLESv2"], src, env)

    for dest in args.outputs:
        name = os.path.basename(dest)
        shutil.copy2(os.path.join(out, name), dest)
        # GN stamps a "./libFoo.dylib" install name, which resolves against the
        # caller's working directory.  Make it relocatable instead.
        subprocess.run(["install_name_tool", "-id", "@rpath/" + name, dest],
                       check=True)
        print("wrote " + dest, flush=True)


if __name__ == "__main__":
    main()

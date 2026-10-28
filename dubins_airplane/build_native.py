"""Build the C++17 backend: python -m dubins_airplane.build_native.

Requires GCC, Clang, MSVC (Developer PowerShell), or Zig. A portable option
is python -m pip install ziglang. No compiler is needed after the build.
"""

import argparse
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

from .native import library_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", help="path to g++, clang++, cl, or zig")
    args = parser.parse_args()
    compiler = (shutil.which(args.compiler) or args.compiler) if args.compiler else None
    if not compiler:
        compiler = next((found for name in ("clang++", "g++", "cl", "zig")
                         if (found := shutil.which(name))), None)
    if not compiler:
        spec = importlib.util.find_spec("ziglang")
        if spec and spec.origin:
            compiler = str(Path(spec.origin).with_name("zig.exe" if sys.platform == "win32" else "zig"))
    if not compiler or not Path(compiler).is_file():
        parser.error("C++ compiler not found; install one or run python -m pip install ziglang")
    source = Path(__file__).resolve().parents[1] / "native"
    build = source / "build"
    build.mkdir(exist_ok=True)
    output = build / library_path().name
    sources = [str(source / name) for name in ("core.cpp", "terrain.cpp", "rrt_star.cpp", "fleet.cpp", "bindings.cpp")]
    name = Path(compiler).stem.lower()
    if name == "cl":
        command = [compiler, "/nologo", "/std:c++17", "/EHsc", "/O2", "/fp:strict", "/LD",
                   "/DDUBINS_BUILDING_LIBRARY",
                   *sources, "/link", "/OUT:" + str(output)]
    else:
        command = [compiler]
        if name == "zig":
            command += ["c++"]
            command += ["-Wno-nullability-completeness"]
            if sys.platform == "win32":
                command += ["-target", "x86_64-windows-gnu"]
        command += ["-std=c++17", "-O3", "-DNDEBUG", "-DDUBINS_BUILDING_LIBRARY",
                    "-fno-fast-math", "-ffp-contract=off",
                    "-Wall", "-Wextra", "-shared"]
        if sys.platform != "win32":
            command += ["-fPIC"]
        command += [*sources, "-o", str(output)]
    print("Compiler:", compiler, flush=True)
    subprocess.run(command, cwd=build, check=True)
    try:
        os.replace(output, library_path())
    except PermissionError:
        parser.error("close Python processes using the native library, then rebuild")
    print("Built:", library_path())


if __name__ == "__main__":
    main()

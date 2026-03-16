from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def pick_default_generator() -> tuple[str, str] | None:
    if sys.platform != "win32":
        return None

    candidates = [
        ("Visual Studio 17 2022", Path(r"C:\Program Files\Microsoft Visual Studio\2022\Community")),
        ("Visual Studio 18 2026", Path(r"C:\Program Files\Microsoft Visual Studio\18\Community")),
    ]

    for generator, install_dir in candidates:
        if install_dir.exists():
            return generator, "x64"

    return None


def find_desktop_root(start: Path) -> Path | None:
    for candidate in (start, *start.parents):
        if (candidate / "CMakeLists.txt").is_file() and (candidate / "app").is_dir():
            return candidate
    return None


def run_command(command: list[str], cwd: Path) -> int:
    print(f"\n> {' '.join(command)}")
    completed = subprocess.run(command, cwd=cwd)
    return completed.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description="Configure and build the desktop project.")
    parser.add_argument(
        "--config",
        default="Debug",
        help="Build configuration for multi-config generators such as Visual Studio. Default: Debug",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Delete the desktop/build directory before configuring.",
    )
    parser.add_argument(
        "--generator",
        help="Optional CMake generator override, for example 'Visual Studio 17 2022'.",
    )
    parser.add_argument(
        "--arch",
        default="x64",
        help="Platform name passed to CMake generators that support -A. Default: x64",
    )
    args = parser.parse_args()

    cwd_root = find_desktop_root(Path.cwd().resolve())
    script_root = find_desktop_root(Path(__file__).resolve().parent)
    desktop_root = cwd_root or script_root

    if desktop_root is None:
        print("Could not find the desktop project root.", file=sys.stderr)
        return 1

    build_dir = desktop_root / "build"

    if args.clean and build_dir.exists():
        print(f"Removing {build_dir}")
        import shutil

        shutil.rmtree(build_dir)

    print(f"Desktop root: {desktop_root}")
    print(f"Build dir: {build_dir}")
    print(f"Configuration: {args.config}")

    configure_command = [
        "cmake",
        "-S",
        str(desktop_root),
        "-B",
        str(build_dir),
    ]

    selected_generator = args.generator
    if selected_generator is None:
        generator_info = pick_default_generator()
        if generator_info is not None:
            selected_generator, detected_arch = generator_info
            if args.arch == "x64":
                args.arch = detected_arch

    if selected_generator:
        print(f"Generator: {selected_generator}")
        configure_command.extend(["-G", selected_generator])
        if args.arch:
            print(f"Architecture: {args.arch}")
            configure_command.extend(["-A", args.arch])

    build_command = [
        "cmake",
        "--build",
        str(build_dir),
        "--config",
        args.config,
    ]

    configure_result = run_command(configure_command, cwd=desktop_root)
    if configure_result != 0:
        return configure_result

    return run_command(build_command, cwd=desktop_root)


if __name__ == "__main__":
    raise SystemExit(main())

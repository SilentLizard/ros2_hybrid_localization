#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}"

ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

CORE_PACKAGE="hybrid_localization_core"
CORE_PACKAGE_DIR="${WORKSPACE_DIR}/src/${CORE_PACKAGE}"
CORE_CMAKE="${CORE_PACKAGE_DIR}/CMakeLists.txt"

BUILD_TYPE="RelWithDebInfo"
CLEAN_BUILD=false
RUN_BUILD=true
RUN_TESTS=true
UPDATE_CMAKE=true
VERBOSE_TESTS=false
SELECTED_PACKAGE=""
PARALLEL_WORKERS=""
BUILD_TESTING=true

usage()
{
  cat <<'EOF'
Usage:
  ./build.sh [options]

Build options:
  --clean                 Delete build/, install/, and log/ before building.
  --debug                 Build with CMAKE_BUILD_TYPE=Debug.
  --release               Build with CMAKE_BUILD_TYPE=Release.
  --relwithdebinfo        Build with CMAKE_BUILD_TYPE=RelWithDebInfo.
  --package NAME          Build and test only the selected ROS package.
  --parallel N            Limit parallel workers to N.

Test options:
  --no-tests              Build but do not execute tests.
  --test-only             Skip compilation and execute tests only.
  --verbose-tests         Show test output directly.
  --no-build-tests        Configure with BUILD_TESTING=OFF.

CMake source-list options:
  --update-cmake          Update marked source and test sections (default).
  --no-update-cmake       Do not modify CMakeLists.txt.

Other:
  -h, --help              Show this help.

Examples:
  ./build.sh
  ./build.sh --clean
  ./build.sh --debug --verbose-tests
  ./build.sh --package hybrid_localization_core
  ./build.sh --test-only --verbose-tests
  ./build.sh --clean --release --no-tests
EOF
}


log()
{
  printf '\n\033[1;34m==> %s\033[0m\n' "$*"
}


error()
{
  printf '\n\033[1;31mERROR: %s\033[0m\n' "$*" >&2
}


on_error()
{
  local exit_code=$?
  local line_number=$1

  error "Command failed at line ${line_number} with exit code ${exit_code}."
  exit "${exit_code}"
}


trap 'on_error ${LINENO}' ERR


require_command()
{
  local command_name=$1

  if ! command -v "${command_name}" >/dev/null 2>&1; then
    error "Required command not found: ${command_name}"
    exit 1
  fi
}


update_cmake_lists()
{
  if [[ ! -f "${CORE_CMAKE}" ]]; then
    error "CMakeLists.txt not found: ${CORE_CMAKE}"
    exit 1
  fi

  log "Updating generated sections in ${CORE_CMAKE}"

  python3 - "${CORE_PACKAGE_DIR}" "${CORE_CMAKE}" "${CORE_PACKAGE}" <<'PY'
from __future__ import annotations

import re
import sys
from pathlib import Path


package_dir = Path(sys.argv[1]).resolve()
cmake_path = Path(sys.argv[2]).resolve()
project_name = sys.argv[3]

source_dir = package_dir / "src"
test_dir = package_dir / "test"


def relative_cpp_files(directory: Path) -> list[str]:
    """Return sorted C++ source paths relative to the package directory."""
    if not directory.is_dir():
        return []

    return sorted(
        path.relative_to(package_dir).as_posix()
        for path in directory.rglob("*.cpp")
        if path.is_file()
    )


def replace_marked_section(
    content: str,
    start_marker: str,
    end_marker: str,
    replacement_body: str,
) -> str:
    """Replace content inside one explicitly marked CMake section."""
    pattern = re.compile(
        rf"(?P<start>^[ \t]*{re.escape(start_marker)}[ \t]*$)"
        rf".*?"
        rf"(?P<end>^[ \t]*{re.escape(end_marker)}[ \t]*$)",
        flags=re.MULTILINE | re.DOTALL,
    )

    match = pattern.search(content)

    if match is None:
        raise RuntimeError(
            f"Could not find marked section:\n"
            f"  {start_marker}\n"
            f"  {end_marker}"
        )

    return (
        content[: match.start()]
        + match.group("start")
        + "\n"
        + replacement_body.rstrip()
        + "\n"
        + match.group("end")
        + content[match.end() :]
    )


source_files = relative_cpp_files(source_dir)
test_files = [
    path
    for path in relative_cpp_files(test_dir)
    if Path(path).name.startswith("test_")
]

if not source_files:
    raise RuntimeError(f"No .cpp files found under {source_dir}")

source_lines = "\n".join(f"  {path}" for path in source_files)

source_section = (
    f"set(HYBRID_LOCALIZATION_CORE_SOURCES\n"
    f"{source_lines}\n"
    f")"
)

test_blocks: list[str] = []

for test_path in test_files:
    test_target = Path(test_path).stem

    test_blocks.append(
        f"  ament_add_gtest({test_target}\n"
        f"    {test_path}\n"
        f"  )\n"
        f"  target_link_libraries({test_target}\n"
        f"    ${{PROJECT_NAME}}\n"
        f"  )"
    )

if test_blocks:
    test_section = "\n\n".join(test_blocks)
else:
    test_section = "  # No test_*.cpp files were found."

original = cmake_path.read_text(encoding="utf-8")

updated = replace_marked_section(
    original,
    "# BEGIN AUTO SOURCES",
    "# END AUTO SOURCES",
    source_section,
)

updated = replace_marked_section(
    updated,
    "# BEGIN AUTO TESTS",
    "# END AUTO TESTS",
    test_section,
)

if updated != original:
    cmake_path.write_text(updated, encoding="utf-8")
    print(f"Updated: {cmake_path}")
else:
    print(f"No changes required: {cmake_path}")

print("Library sources:")
for source_file in source_files:
    print(f"  - {source_file}")

print("Test sources:")
for test_file in test_files:
    print(f"  - {test_file}")
PY
}


source_ros_environment()
{
  if [[ ! -f "${ROS_SETUP}" ]]; then
    error "ROS setup file not found: ${ROS_SETUP}"
    error "Expected ROS distribution: ${ROS_DISTRO_NAME}"
    exit 1
  fi

  # A previously sourced workspace may refer to install directories removed by
  # --clean. Remove those overlay paths before sourcing the stable ROS underlay.
  unset AMENT_PREFIX_PATH || true
  unset CMAKE_PREFIX_PATH || true
  unset COLCON_PREFIX_PATH || true

  # ROS setup scripts legitimately reference variables that may not yet exist.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
}


run_clean()
{
  log "Removing generated workspace directories"

  rm -rf \
    "${WORKSPACE_DIR}/build" \
    "${WORKSPACE_DIR}/install" \
    "${WORKSPACE_DIR}/log"

  rm -f "${WORKSPACE_DIR}/compile_commands.json"
}


run_rosdep()
{
  log "Resolving ROS dependencies"

  rosdep install \
    --from-paths "${WORKSPACE_DIR}/src" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO_NAME}" \
    -r \
    -y
}


run_build()
{
  log "Building workspace with CMake build type ${BUILD_TYPE}"

  local command=(
    colcon build
    --symlink-install
  )

  if [[ -n "${SELECTED_PACKAGE}" ]]; then
    command+=(
      --packages-select
      "${SELECTED_PACKAGE}"
    )
  fi

  if [[ -n "${PARALLEL_WORKERS}" ]]; then
    command+=(
      --parallel-workers
      "${PARALLEL_WORKERS}"
    )
  fi

  command+=(
    --cmake-args
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    "-DBUILD_TESTING=${BUILD_TESTING}"
  )

  "${command[@]}"
}


run_tests()
{
  log "Running tests"

  local command=(
    colcon test
  )

  if [[ -n "${SELECTED_PACKAGE}" ]]; then
    command+=(
      --packages-select
      "${SELECTED_PACKAGE}"
    )
  fi

  if [[ "${VERBOSE_TESTS}" == true ]]; then
    command+=(
      --event-handlers
      console_direct+
    )
  fi

  "${command[@]}"

  log "Collecting test results"
  colcon test-result --verbose
}


create_compile_commands_link()
{
  local selected="${SELECTED_PACKAGE:-${CORE_PACKAGE}}"
  local compile_commands=(
    "${WORKSPACE_DIR}/build/${selected}/compile_commands.json"
  )

  if [[ -f "${compile_commands[0]}" ]]; then
    ln -sfn \
      "${compile_commands[0]}" \
      "${WORKSPACE_DIR}/compile_commands.json"

    log "Created compile_commands.json link for ${selected}"
  else
    log "No compile_commands.json found for ${selected}"
  fi
}


while (($# > 0)); do
  case "$1" in
    --clean)
      CLEAN_BUILD=true
      ;;

    --debug)
      BUILD_TYPE="Debug"
      ;;

    --release)
      BUILD_TYPE="Release"
      ;;

    --relwithdebinfo)
      BUILD_TYPE="RelWithDebInfo"
      ;;

    --package)
      if (($# < 2)); then
        error "--package requires a package name"
        exit 2
      fi

      SELECTED_PACKAGE="$2"
      shift
      ;;

    --parallel)
      if (($# < 2)); then
        error "--parallel requires a positive integer"
        exit 2
      fi

      if [[ ! "$2" =~ ^[1-9][0-9]*$ ]]; then
        error "--parallel must be a positive integer"
        exit 2
      fi

      PARALLEL_WORKERS="$2"
      shift
      ;;

    --no-tests)
      RUN_TESTS=false
      ;;

    --test-only)
      RUN_BUILD=false
      RUN_TESTS=true
      ;;

    --verbose-tests)
      VERBOSE_TESTS=true
      ;;

    --no-build-tests)
      BUILD_TESTING=false
      RUN_TESTS=false
      ;;

    --update-cmake)
      UPDATE_CMAKE=true
      ;;

    --no-update-cmake)
      UPDATE_CMAKE=false
      ;;

    -h | --help)
      usage
      exit 0
      ;;

    *)
      error "Unknown option: $1"
      usage
      exit 2
      ;;
  esac

  shift
done


main()
{
  require_command python3
  require_command colcon
  require_command rosdep

  cd "${WORKSPACE_DIR}"

  source_ros_environment

  if [[ "${CLEAN_BUILD}" == true ]]; then
    run_clean

    # Cleaning removes a potentially sourced workspace overlay. Restore only
    # the system ROS installation before continuing.
    source_ros_environment
  fi

  if [[ "${UPDATE_CMAKE}" == true ]]; then
    update_cmake_lists
  fi

  if [[ "${RUN_BUILD}" == true ]]; then
    run_rosdep
    run_build
    create_compile_commands_link
  fi

  if [[ "${RUN_TESTS}" == true ]]; then
    run_tests
  fi

  log "Completed successfully"
}


main
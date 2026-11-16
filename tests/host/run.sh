#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
if [[ -z "${TINYGPS_SRC:-}" ]]; then
  TINYGPS_SRC="$(python3 - <<'PYGPS'
from pathlib import Path
for props in Path('.pio/libdeps').glob('*/TinyGPSPlus*/library.properties'):
    if 'version=1.0.3' in props.read_text():
        print(props.parent / 'src')
        break
PYGPS
)"
fi
if [[ ! -f "$TINYGPS_SRC/TinyGPS++.cpp" ]]; then
  echo 'Build a PlatformIO example first, or set TINYGPS_SRC to the verified TinyGPSPlus 1.0.3 src directory.' >&2
  exit 1
fi
test_build="$(mktemp -d)"
trap 'rm -rf "$test_build"' EXIT
"${CXX:-clang++}" -std=c++17 -DARDUINO=10800 -Wall -Wextra -Werror \
  -Wno-implicit-fallthrough -Wno-unused-parameter \
  -fsanitize=address,undefined -fno-omit-frame-pointer -g \
  -Itests/host/stubs -Isrc -I"$TINYGPS_SRC" \
  src/DuckGPS.cpp src/DuckGPSFix.cpp "$TINYGPS_SRC/TinyGPS++.cpp" tests/host/test.cpp \
  -o "$test_build/gps-tests"
UBSAN_OPTIONS=halt_on_error=1 "$test_build/gps-tests"

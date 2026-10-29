#!/usr/bin/env bash
# Build a real ARM64/x86_64 DUFOMap Python backend without sudo or system pip.
# Usage: scripts/install_dufomap_backend.sh [--smoke]
set -euo pipefail
if [[ ${1:-} != '' && ${1:-} != --smoke ]]; then
  echo 'Usage: install_dufomap_backend.sh [--smoke]' >&2
  exit 2
fi
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CACHE=${DUFOMAP_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/lio_sam_hesai/dufomap}
VENV=${DUFOMAP_VENV_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/lio_sam_hesai/dufomap-venv}
PYTHON=${DUFOMAP_BASE_PYTHON:-python3}
BRIDGE_SOURCE=$(cd -- "$SCRIPT_DIR/dufomap_native" && pwd -P)
BRIDGE_PATH_HASH=$(printf '%s' "$BRIDGE_SOURCE" | sha256sum | cut -c1-16)
BUILD_DIR="$CACHE/build-$BRIDGE_PATH_HASH"
DUFO_REV=9e239ddd5995136e14f5212f33382a6ebc59e518
UFO_REV=fcb20f445d257280c039c07ab756c9d5b37ebd78
mkdir -p "$CACHE/sources" "$CACHE/logs" "$CACHE/licenses" "$(dirname -- "$VENV")"
for task_command in git cmake c++ pkg-config; do
  command -v "$task_command" >/dev/null || { echo "Missing build prerequisite: $task_command" >&2; exit 1; }
done
# Reuse the exploratory source downloads, preserving their complete history.
source_checkout() {
  local name=$1 url=$2 revision=$3 legacy=$4 target="$CACHE/sources/$1"
  if [[ ! -d "$target/.git" ]]; then
    if [[ -d "$CACHE/legacy/$legacy/.git" ]]; then
      git clone --quiet --no-hardlinks "$CACHE/legacy/$legacy" "$target"
      git -C "$target" remote set-url origin "$url"
    else
      git init --quiet "$target"
      git -C "$target" remote add origin "$url"
    fi
  fi
  if ! git -C "$target" cat-file -e "$revision^{commit}" 2>/dev/null; then
    git -C "$target" fetch --quiet --depth 1 origin "$revision"
  fi
  if [[ $(git -C "$target" rev-parse HEAD 2>/dev/null || true) != "$revision" ]]; then
    git -C "$target" checkout --quiet --detach "$revision"
  fi
  git -C "$target" diff --quiet HEAD -- || { echo "Modified upstream source: $target" >&2; exit 1; }
}
source_checkout dufomap https://github.com/KTH-RPL/dufomap.git "$DUFO_REV" .dufomap_src
source_checkout ufomap https://github.com/UnknownFreeOccupied/ufomap.git "$UFO_REV" .ufomap_src
[[ $(git -C "$CACHE/sources/dufomap" ls-tree HEAD ufomap | awk '{print $3}') == "$UFO_REV" ]] || { echo 'UFOMap revision does not match DUFOMap submodule' >&2; exit 1; }
cp "$CACHE/sources/dufomap/LICENSE" "$CACHE/licenses/DUFOMap-BSD-3-Clause.txt"
cp "$CACHE/sources/ufomap/LICENSE" "$CACHE/licenses/UFOMap-LICENSE.txt"
if [[ -f "$CACHE/sources/ufomap/LICENSE.md" ]]; then
  cp "$CACHE/sources/ufomap/LICENSE.md" "$CACHE/licenses/UFOMap-LICENSE.md"
fi
# Ubuntu liblzf development files can be unpacked privately; no package install.
# TBB, LZ4, OpenMP and Python development headers must already be installed.
LZF_PREFIX="$CACHE/deps/root/usr"
if [[ ! -f /usr/include/liblzf/lzf.h && ! -f "$LZF_PREFIX/include/liblzf/lzf.h" ]]; then
  mkdir -p "$CACHE/deps"
  for package in liblzf-dev liblzf1; do
    task_deb="$CACHE/deps/${package}_3.6-3_$(dpkg --print-architecture).deb"
    task_legacy="$CACHE/legacy/.dufomap_native_work/deps/$(basename -- "$task_deb")"
    if [[ ! -f "$task_deb" ]]; then
      if [[ -f "$task_legacy" ]]; then
        cp "$task_legacy" "$task_deb"
      else
        (cd "$CACHE/deps" && apt-get download "$package=3.6-3") >&2
      fi
    fi
    dpkg-deb -x "$task_deb" "$CACHE/deps/root"
  done
  cp "$CACHE/deps/root/usr/share/doc/liblzf1/copyright" "$CACHE/licenses/liblzf-copyright.txt"
fi
if [[ ! -x "$VENV/bin/python" ]]; then
  "$PYTHON" -m venv "$VENV"
fi
# Keep Python packages private; reuse an installed NumPy copy offline when available.
sed -i 's/^include-system-site-packages = true/include-system-site-packages = false/' "$VENV/pyvenv.cfg"
if ! "$VENV/bin/python" -c 'import numpy; assert int(numpy.__version__.split(".")[0]) < 2' 2>/dev/null; then
  if "$PYTHON" -c 'import numpy; assert int(numpy.__version__.split(".")[0]) < 2' 2>/dev/null; then
    "$PYTHON" - "$VENV" <<'PYTHON_COPY'
import importlib.metadata
import pathlib
import shutil
import sys
import numpy
venv = pathlib.Path(sys.argv[1])
site = venv / "lib" / ("python%d.%d" % sys.version_info[:2]) / "site-packages"
source = pathlib.Path(numpy.__file__).parent
shutil.copytree(source, site / "numpy", dirs_exist_ok=True)
for sibling in source.parent.glob("numpy.libs"):
    shutil.copytree(sibling, site / sibling.name, dirs_exist_ok=True)
metadata = pathlib.Path(importlib.metadata.distribution("numpy")._path)
if metadata.is_dir():
    shutil.copytree(metadata, site / metadata.name, dirs_exist_ok=True)
PYTHON_COPY
  else
    "$VENV/bin/python" -m pip install --disable-pip-version-check numpy==1.26.4 > "$CACHE/logs/pip.log" 2>&1 || { cat "$CACHE/logs/pip.log" >&2; exit 1; }
  fi
fi
# Prefer locally installed build headers. Install only into this venv if absent.
PYBIND_ARGS=()
if "$VENV/bin/python" -c 'import pybind11' 2>/dev/null; then
  PYBIND_ARGS+=("-Dpybind11_DIR=$("$VENV/bin/python" -m pybind11 --cmakedir)")
elif [[ ! -f /usr/include/pybind11/pybind11.h ]]; then
  "$VENV/bin/python" -m pip install --disable-pip-version-check pybind11==2.13.6 > "$CACHE/logs/pip-pybind11.log" 2>&1 || { cat "$CACHE/logs/pip-pybind11.log" >&2; exit 1; }
  PYBIND_ARGS+=("-Dpybind11_DIR=$("$VENV/bin/python" -m pybind11 --cmakedir)")
fi
SITE=$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["platlib"])')
cmake -S "$BRIDGE_SOURCE" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release -DUFOMAP_SOURCE="$CACHE/sources/ufomap" \
  -DPython3_EXECUTABLE="$VENV/bin/python" "${PYBIND_ARGS[@]}" \
  -DCMAKE_PREFIX_PATH="$LZF_PREFIX" -DCMAKE_INSTALL_RPATH="$LZF_PREFIX/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)" \
  > "$CACHE/logs/configure.log" 2>&1 || { cat "$CACHE/logs/configure.log" >&2; exit 1; }
cmake --build "$BUILD_DIR" --parallel "${DUFOMAP_BUILD_JOBS:-2}" > "$CACHE/logs/build.log" 2>&1 || { tail -80 "$CACHE/logs/build.log" >&2; exit 1; }
cmake --install "$BUILD_DIR" --prefix "$SITE" > "$CACHE/logs/install.log" 2>&1
mkdir -p "$SITE/dufomap_native_licenses"
cp -a "$CACHE/licenses/." "$SITE/dufomap_native_licenses/"
"$VENV/bin/python" -c 'from dufomap import dufomap; import dufomap as module; print(module.__backend__)' >&2
if [[ ${1:-} == --smoke ]]; then
  "$VENV/bin/python" "$SCRIPT_DIR/dufomap_native/smoke.py" | tee "$CACHE/logs/smoke.json" >&2
fi
printf '%s\n' "$VENV/bin/python"

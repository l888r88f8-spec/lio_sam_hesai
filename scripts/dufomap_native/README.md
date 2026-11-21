This is a thin Python binding, not a substitute dynamic filter. It instantiates
`ufo::Map<SEEN_FREE | REFLECTION | LABEL>`, runs upstream `insertPointCloud`,
propagates with `propagateModified`, and classifies with `seenFree`, exactly as
the official DUFOMap CLI does. Label 0 means static/unknown; 1 means seen-free.
Clustering is not exposed: `oncePropagateCluster(if_cluster=True)` fails clearly.
The production adapter uses `if_cluster=False`.

Sources and licenses:

- https://github.com/KTH-RPL/dufomap at
  `9e239ddd5995136e14f5212f33382a6ebc59e518`, BSD 3-Clause.
- Its exact UFOMap submodule, https://github.com/UnknownFreeOccupied/ufomap at
  `fcb20f445d257280c039c07ab756c9d5b37ebd78`, BSD 3-Clause.
- The installer preserves upstream license texts alongside the installed module
  in `dufomap_native_licenses/` and in the external dependency cache.

From the project root, run `scripts/install_dufomap_backend.sh --smoke`.
It prints the absolute Python path on stdout. Source downloads, build files,
logs and private liblzf dependencies live under
`~/.cache/lio_sam_hesai/dufomap`; the Python environment lives under
`~/.local/share/lio_sam_hesai/dufomap-venv`. `DUFOMAP_CACHE_DIR`,
`DUFOMAP_VENV_DIR`, `DUFOMAP_BASE_PYTHON`, and `DUFOMAP_BUILD_JOBS` override
those defaults. Repeating the command reuses pinned sources and incremental
build output. No sudo, system Python writes or workspace dependency trees.

Build prerequisites are CMake >= 3.17, a C++20 compiler, Python development
headers/venv, TBB, OpenMP, LZ4, pybind11, and NumPy. The installer can privately
unpack Ubuntu liblzf 3.6-3 packages if its headers are missing. It reuses an
installed NumPy by copying its package into the private venv (without enabling
system-site-packages), or installs NumPy 1.26.4 there. It uses installed pybind11
build headers, or installs pybind11 2.13.6 into the private venv. Fetching missing
sources/packages requires network access; cached sources and installed build
dependencies work offline. It does not change apt package state.

ARM64 portability requires only an empty `immintrin.h` include shim: upstream
unconditionally includes that x86 header, while every actual BMI2 use is guarded
by `__BMI2__`. The upstream portable implementations remain in use.

The pinned upstream `getMisses` serial branch is unimplemented. Consequently
`num_threads=1` is rejected, while `0` selects between 2 and 8 threads. Values
>=2 are passed through. `d_p` supports integers 0..4 (the upstream template
dispatch range), `d_s` is the hit inflation distance in meters, and resolution
is the voxel size. Range selection belongs to the production adapter; the
bridge applies no additional minimum-range cutoff. Dense enough ray coverage
is necessary for unknown-space erosion, so a few isolated points may all
remain static even when the native backend is working correctly.

`smoke.py` uses four actual local-frame scans with a translated sensor pose, a
9801-point static wall, moving foreground points, removed wall rays behind the
foreground point, and unknown queries behind the wall. It asserts an observed
foreground point changes from label 0 to 1 after later rays clear its location,
all wall points stay static, and unseen occluded queries remain static. Explicit two-thread and automatic-thread configurations with `d_p=1`, plus
the shipping default of automatic threads with `d_p=2`, must pass. The
unsafe one-thread configuration must reject. Use `--output-dir PATH` to retain
the deterministic scene arrays and JSON result.

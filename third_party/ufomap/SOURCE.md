# Vendored official DUFOMap / UFOMap headers

DUFOMap upstream: https://github.com/KTH-RPL/dufomap

Fixed revision: `9e239ddd5995136e14f5212f33382a6ebc59e518`.

UFOMap upstream: https://github.com/UnknownFreeOccupied/ufomap

Fixed revision: `fcb20f445d257280c039c07ab756c9d5b37ebd78`.

The native adapter reproduces the upstream DUFOMap C++ core call sequence
(`insertPointCloud` with the fixed integration parameters, then
`propagateModified`, then `seenFree`) without pybind11 or Python. Only the
transitive header closure required by `ufo/map/ufomap.hpp` and
`ufo/map/integration/integration.hpp` is retained. Files are byte-for-byte
upstream copies; `SHA256SUMS` records them. `surfel_node.hpp` is referenced by
an optional include and is not part of the compiled closure.

`arm64/immintrin.h` is a local portability shim, not an upstream file: upstream
includes the x86 header but gates every intrinsic on `__BMI2__`, so the empty
shim selects the portable path. `compat/liblzf/lzf.h` is the official liblzf
1.5 header (BSD-2-Clause) referenced by `ufo/map/point_cloud.hpp`; upstream's
compressed PCD writer is still a TODO and calls no lzf symbol, so the adapter
does not link liblzf. TBB and OpenMP are supplied by the existing ROS build
environment. LZ4 is likewise not referenced by the retained headers.

The BSD-3-Clause UFOMap license is in `LICENSE`; the liblzf notice is in
`LICENSE.liblzf`.

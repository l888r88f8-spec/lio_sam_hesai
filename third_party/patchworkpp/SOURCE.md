# Vendored Patchwork++ C++ core

Upstream: https://github.com/url-kaist/patchwork-plusplus

Fixed revision: `3e6903a1d5537a4cc2ace897b0bbb98a92d6014c`.

Source archive: https://github.com/url-kaist/patchwork-plusplus/archive/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c.tar.gz

Archive SHA-256: `fcd4928a99d29b1556b14c232113c28c0910c66cdd6ada05a079723cf05190a1`.

License: BSD 2-Clause; the original complete text is in `LICENSE`.

Only these original files are retained, with their upstream paths:

- `cpp/common/include/patchwork/types.h`
- `cpp/common/include/patchwork/plane_fit.h`
- `cpp/common/src/plane_fit.cpp`
- `cpp/patchworkpp/include/patchwork/patchworkpp.h`
- `cpp/patchworkpp/src/patchworkpp.cpp`
- `LICENSE`

All six files are byte-for-byte upstream copies. There are no algorithm or
compatibility patches. `SHA256SUMS` records the vendored file checksums.
Open3D, Python bindings, examples, downloads, upstream build scripts, and the
separate classic Patchwork algorithm are not included. The C++ core only needs
Eigen. The parent project owns build targets and links its PCL adapter.

The adapter in `src/common/patchworkpp_ground_segmenter.cpp` configures upstream
`patchwork::Params`, converts finite XYZ input rows into an XYZI matrix, calls
`patchwork::PatchWorkpp::estimateGround`, and maps `getGroundIndices()` back to
the original input rows. It never merges or changes original points. It sorts
the resulting indices into input order. Upstream sensor-height and threshold
adaptation remain enabled and persist across successive calls to one instance;
online and export streams must use separate instances.

The input is an individual lidar-frame scan with its original XYZI fields.
Ground selection does not need ring or time. Invalid XYZ points are ineligible,
and empty/all-invalid scans return an empty result without resetting temporal
state. RNR is disabled by default because uncalibrated intensity is unsafe for
reflective-noise classification. When disabled, nonfinite intensity does not
affect geometric classification. With RNR explicitly enabled, a valid XYZ row
with nonfinite intensity raises a clear error; no row is silently removed or
assigned synthetic intensity. Upstream range selection uses XY radial distance
`min_range < radius <= max_range`.

The upstream constructor prints an initialization line even with verbose mode
disabled; that behavior is retained rather than patching the core.

Validation preserves upstream adaptive behavior rather than adding a replacement
classifier. A direct-core probe on an exactly zero-noise 10172-point horizontal
plane returned 10100 ground indices initially and 6064 on each of the next two
frames. The empty-input regression therefore compares its state with an
uninterrupted scan stream, rather than assuming adaptive recall is constant on
that degenerate synthetic scene. Separate sloped-plane and obstacle scenes
verify ground/obstacle selection. No upstream fix is included.

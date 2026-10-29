// Thin Python bridge to the unmodified KTH-RPL DUFOMap UFOMap core.
// The dynamic label is the upstream CLI's map.seenFree(point) predicate.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/ufomap.hpp>
#include <algorithm>
#include <cmath>
#include <thread>

namespace py = pybind11;
using Array = py::array_t<float, py::array::c_style | py::array::forcecast>;
using Pose = py::array_t<double, py::array::c_style | py::array::forcecast>;
using Map = ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL>;

class DUFOMap {
  Map map_;
  ufo::IntegrationParams params_;
  double limit_;
  static double resolution(double value) {
    if (!std::isfinite(value) || value <= 0) throw py::value_error("resolution must be finite and positive");
    return value;
  }
  ufo::PointCloud cloud(Array xyz, Pose pose, bool transform, ufo::Point& origin) const {
    if (xyz.ndim() != 2 || xyz.shape(1) != 3) throw py::value_error("cloud must have shape (N, 3)");
    if (pose.ndim() != 2 || pose.shape(0) != 4 || pose.shape(1) != 4) throw py::value_error("pose must have shape (4, 4)");
    auto p = pose.unchecked<2>();
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
      if (!std::isfinite(p(r,c))) throw py::value_error("pose must be finite");
    if (std::abs(p(3,0)) > 1e-8 || std::abs(p(3,1)) > 1e-8 || std::abs(p(3,2)) > 1e-8 || std::abs(p(3,3)-1) > 1e-8)
      throw py::value_error("pose must be a homogeneous transform");
    auto check = [&](double x, double y, double z) {
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          std::abs(x) >= limit_ || std::abs(y) >= limit_ || std::abs(z) >= limit_)
        throw py::value_error("point outside finite DUFOMap coordinate range");
    };
    check(p(0,3), p(1,3), p(2,3));
    origin = ufo::Point(p(0,3), p(1,3), p(2,3));
    auto a = xyz.unchecked<2>();
    ufo::PointCloud result;
    result.reserve(xyz.shape(0));
    for (py::ssize_t i = 0; i < xyz.shape(0); ++i) {
      double x = a(i,0), y = a(i,1), z = a(i,2);
      if (transform) {
        auto tx = p(0,0)*x+p(0,1)*y+p(0,2)*z+p(0,3);
        auto ty = p(1,0)*x+p(1,1)*y+p(1,2)*z+p(1,3);
        z = p(2,0)*x+p(2,1)*y+p(2,2)*z+p(2,3); x = tx; y = ty;
      }
      check(x,y,z);
      result.push_back(ufo::Point(x,y,z));
    }
    return result;
  }
public:
  DUFOMap(double res, double ds, int dp, int threads)
    : map_(resolution(res), 20), limit_(std::ldexp(res, 19)) {
    if (!std::isfinite(ds) || ds < 0 || dp < 0 || dp > 4 || threads < 0 || threads == 1)
      throw py::value_error("d_s must be nonnegative, d_p in [0, 4], num_threads must be 0 (automatic) or >= 2");
    // Match KTH-RPL/dufomap assets/config.toml. Keep automatic threads bounded
    // instead of the upstream default of 8 * all CPUs on large hosts.
    params_.inflate_hits_dist = ds;
    params_.inflate_unknown = dp;
    params_.inflate_unknown_compensation = true;
    params_.ray_passthrough_hits = false;
    params_.min_range = 0.0f; // The production adapter applies its configured range gate.
    params_.num_threads = threads ? threads : std::max(2u, std::min(8u, std::thread::hardware_concurrency()));
    // Upstream getMisses has no serial implementation; never select it.
    params_.parallel = true;
  }
  void run(Array xyz, Pose pose, bool transform) {
    ufo::Point origin;
    auto points = cloud(xyz, pose, transform, origin);
    if (points.empty()) return;
    // Keep the GIL: each instance is mutable and must not be called concurrently.
    ufo::insertPointCloud(map_, points, origin, params_, false);
  }
  void propagate(bool propagate, bool cluster) {
    if (cluster) throw py::value_error("this bridge supports if_cluster=False only");
    if (propagate) map_.propagateModified();
  }
  py::array_t<std::uint8_t> segment(Array xyz, Pose pose, bool transform) {
    ufo::Point origin;
    auto points = cloud(xyz, pose, transform, origin);
    py::array_t<std::uint8_t> labels(points.size());
    auto out = labels.mutable_unchecked<1>();
    for (std::size_t i = 0; i < points.size(); ++i) out(i) = map_.seenFree(points[i]) ? 1 : 0;
    return labels;
  }
};
PYBIND11_MODULE(dufomap, m) {
  m.doc() = "Native bridge to KTH-RPL DUFOMap's pinned UFOMap core (BSD-3-Clause)";
  m.attr("__version__") = "native-bridge-1+dufomap.9e239ddd.ufomap.fcb20f4";
  m.attr("__backend_kind__") = "native-pybind-upstream-core";
  m.attr("__backend__") = "KTH-RPL/dufomap@9e239ddd5995136e14f5212f33382a6ebc59e518";
  m.attr("__ufomap_revision__") = "fcb20f445d257280c039c07ab756c9d5b37ebd78";
  py::class_<DUFOMap>(m, "dufomap")
    .def(py::init<double,double,int,int>(), py::arg("resolution"), py::arg("d_s"), py::arg("d_p"), py::arg("num_threads")=0)
    .def("run", &DUFOMap::run, py::arg("cloud"), py::arg("pose"), py::arg("cloud_transform")=true)
    .def("oncePropagateCluster", &DUFOMap::propagate, py::arg("if_propagate")=true, py::arg("if_cluster")=false)
    .def("segment", &DUFOMap::segment, py::arg("cloud"), py::arg("pose"), py::arg("cloud_transform")=true);
}

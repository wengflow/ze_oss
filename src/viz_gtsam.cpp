#include <ze/visualization/viz_gtsam.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <ze/visualization/viz_interface.h>

namespace ze {

void drawGtsamPoint3(
    Visualizer& visualizer,
    const gtsam::Values& values,
    const char key_prefix,
    const std::string& ns,
    const size_t id,
    const Color& color,
    const FloatType size)
{
  auto gtsam_points = values.filter<gtsam::Point3>(gtsam::Symbol::ChrTest(key_prefix));
  const size_t n_points = gtsam_points.size();
  Positions points(3, n_points);
  size_t i = 0;
  for(const auto& it : gtsam_points)
  {
    points.col(i++) = it.value.vector();
  }
  visualizer.drawPoints(ns, id, points, color, size);
}

void drawGtsamPose3(
    Visualizer& visualizer,
    const gtsam::Values& values,
    const char key_prefix,
    const std::string& ns,
    const size_t id,
    const FloatType size)
{
  auto gtsam_poses = values.filter<gtsam::Pose3>(gtsam::Symbol::ChrTest(key_prefix));
  const size_t n_poses = gtsam_poses.size();
  TransformationVector poses;
  poses.reserve(n_poses);
  for(const auto& it : gtsam_poses)
  {
    poses.push_back(Transformation(Quaternion(it.value.rotation().matrix()),
                                   it.value.translation().vector()));
  }
  visualizer.drawCoordinateFrames(ns, id, poses, size);
}

} // namespace ze

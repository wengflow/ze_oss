#include <string>
#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <ze/common/file_utils.h>
#include <ze/common/csv_trajectory.h>
#include <ze/trajectory_analysis/kitti_evaluation.h>

DEFINE_string(data_dir, ".", "Path to data");
DEFINE_string(filename_es, "traj_es.csv", "Filename of estimated trajectory.");
DEFINE_string(filename_gt, "traj_gt.csv", "Filename of groundtruth trajectory.");
DEFINE_string(filename_result_prefix, "traj_relative_errors", "Filename prefix of result.");
DEFINE_double(offset_sec, 0.0, "time offset added to the timestamps of the estimate");
DEFINE_double(max_difference_sec, 0.02, "maximally allowed time difference for matching entries");

DEFINE_double(segment_length, 50, "Segment length of relative error evaluation. [meters]");
DEFINE_double(skip_frames, 10, "Number of frames to skip between evaluation.");

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Load groundtruth.
  ze::PoseSeries gt_data;
  gt_data.load(ze::joinPath(FLAGS_data_dir, FLAGS_filename_gt));

  // Load estimate data.
  ze::SWEResultSeries es_data;
  es_data.load(ze::joinPath(FLAGS_data_dir, FLAGS_filename_es));
  ze::StampedTransformationVector es_stamped_poses =
      es_data.getStampedTransformationVector();

  // Now loop through all estimate stamps and find closest groundtruth-stamp.
  ze::TransformationVector es_poses, gt_poses;
  es_poses.reserve(es_stamped_poses.size());
  gt_poses.reserve(es_stamped_poses.size());
  {
    int64_t offset_nsec = ze::secToNanosec(FLAGS_offset_sec);
    int64_t max_diff_nsec = ze::secToNanosec(FLAGS_max_difference_sec);
    int n_skipped = 0;
    VLOG(1) << "Associated timestamps of " << es_stamped_poses.size() << " poses...";
    for(const auto& it : es_stamped_poses)
    {
      bool success;
      int64_t gt_stamp;
      ze::Vector7 gt_pose_data;
      std::tie(gt_stamp, gt_pose_data, success) =
          gt_data.getBuffer().getNearestValue(it.first + offset_nsec);
      CHECK(success);
      if(std::abs(gt_stamp - it.first + offset_nsec) > max_diff_nsec)
      {
        ++n_skipped;
        continue;
      }

      es_poses.push_back(it.second);
      gt_poses.push_back(ze::PoseSeries::getTransformationFromVec7(gt_pose_data));
    }
    VLOG(1) << "...done. Found " << es_poses.size() << " matches.";
  }


  // Kitti evaluation
  VLOG(1) << "Computing relative errors...";
  std::vector<ze::RelativeError> errors =
      ze::calcSequenceErrors(gt_poses, es_poses, FLAGS_segment_length, FLAGS_skip_frames);
  VLOG(1) << "...done";

  // Write result to file
  std::string filename_result =
      FLAGS_filename_result_prefix + "_" + std::to_string(FLAGS_segment_length) + ".csv";
  VLOG(1) << "Write result to file: " << ze::joinPath(FLAGS_data_dir, filename_result);
  std::ofstream fs;
  ze::openOutputFileStream(ze::joinPath(FLAGS_data_dir, filename_result), &fs);
  fs << "# First frame index, rot. err. rel., transl. err. rel., length, num frames\n";
  for(const ze::RelativeError& err : errors)
  {
    fs << err.first_frame << ", "
       << err.rot_error << ", "
       << err.tran_error << ", "
       << err.len << ", "
       << err.num_frames << "\n";
  }
  fs.close();
  VLOG(1) << "Finished.";

  return 0;
}

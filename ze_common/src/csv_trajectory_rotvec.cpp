// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ze/common/csv_trajectory_rotvec.hpp>

namespace ze {

int64_t CSVTrajectoryRotVec::getTimeStamp(const std::string& ts_str) const
{
  return std::stoll(ts_str);
}

void CSVTrajectoryRotVec::readHeader(const std::string& in_file_path)
{
  in_str_.open(in_file_path);
  CHECK(in_str_.is_open());
  if(!header_.empty())
  {
    std::string line;
    getline(in_str_, line);
    CHECK_EQ(line.substr(0, header_.size()), header_);
  }
}

Vector3 CSVTrajectoryRotVec::readTranslation(const std::vector<std::string>& items)
{
  return Vector3(
        std::stod(items[order_.find("tx")->second]),
        std::stod(items[order_.find("ty")->second]),
        std::stod(items[order_.find("tz")->second]));
}

Vector3 CSVTrajectoryRotVec::readOrientation(const std::vector<std::string>& items)
{
  Vector3 q(
        std::stod(items[order_.find("rx")->second]),
        std::stod(items[order_.find("ry")->second]),
        std::stod(items[order_.find("rz")->second]));
  return q;
}

Vector6 CSVTrajectoryRotVec::readPose(const std::vector<std::string>& items)
{
  Vector6 pose;
  pose << readTranslation(items), readOrientation(items);
  return pose;
}

PositionSeriesRotVec::PositionSeriesRotVec()
{
  order_["ts"] = 0;
  order_["tx"] = 1;
  order_["ty"] = 2;
  order_["tz"] = 3;

  header_ = "# timestamp, x, y, z";
  num_tokens_in_line_ = 4u;
}

void PositionSeriesRotVec::load(const std::string& in_file_path)
{
  readHeader(in_file_path);
  std::string line;
  while(getline(in_str_, line))
  {
    if('%' != line.at(0) && '#' != line.at(0))
    {
      std::vector<std::string> items = ze::splitString(line, delimiter_);
      CHECK_GE(items.size(), num_tokens_in_line_);
      int64_t stamp = getTimeStamp(items[order_.find("ts")->second]);
      Vector3 position = readTranslation(items);
      position_buf_.insert(stamp, position);
    }
  }
}

const Buffer<real_t, 3>& PositionSeriesRotVec::getBuffer() const
{
  return position_buf_;
}

Buffer<real_t, 3>& PositionSeriesRotVec::getBuffer()
{
  return position_buf_;
}

PoseSeriesRotVec::PoseSeriesRotVec()
{
  order_["ts"] = 0;
  order_["tx"] = 1;
  order_["ty"] = 2;
  order_["tz"] = 3;
  order_["rx"] = 4;
  order_["ry"] = 5;
  order_["rz"] = 6;

  header_ = "# timestamp, x, y, z, rx, ry, rz";
  num_tokens_in_line_ = 7u;
}

void PoseSeriesRotVec::load(const std::string& in_file_path)
{
  readHeader(in_file_path);
  std::string line;
  while(getline(in_str_, line))
  {
    if('%' != line.at(0) && '#' != line.at(0) && 't' != line.at(0))
    {
      std::vector<std::string> items = ze::splitString(line, delimiter_);
      CHECK_GE(items.size(), num_tokens_in_line_);
      int64_t stamp = getTimeStamp(items[order_.find("ts")->second]);
      Vector6 pose = readPose(items);
      pose_buf_.insert(stamp, pose);
    }
  }
}

const Buffer<real_t, 6>& PoseSeriesRotVec::getBuffer() const
{
  return pose_buf_;
}

Buffer<real_t, 6>& PoseSeriesRotVec::getBuffer()
{
  return pose_buf_;
}

StampedRotVecTransformations PoseSeriesRotVec::getStampedRotVecTransformations()
{
  pose_buf_.lock();
  auto& data = pose_buf_.data();

  StampedRotVecTransformations stamped_tfs;
  Stamps stamps(data.size());
  RotVecTransformations tfs(6, data.size());

  int index = 0;
  for(const auto& it : data)
  {
    stamps(index) = it.first;
    tfs.col(index) = it.second;
    ++index;
  }
  stamped_tfs = std::make_pair(stamps, tfs);
  
  pose_buf_.unlock();
  return stamped_tfs;
}

} // ze namespace

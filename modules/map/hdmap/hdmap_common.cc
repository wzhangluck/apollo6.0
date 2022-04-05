/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "modules/map/hdmap/hdmap_common.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/map/hdmap/hdmap_impl.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace hdmap {
namespace {
using apollo::common::PointENU;
using apollo::common::math::Vec2d;

// Minimum error in lane segmentation.
// const double kSegmentationEpsilon = 0.2;

// Minimum distance to remove duplicated points.
constexpr double kDuplicatedPointsEpsilon = 1e-7;

// Margin for comparation
constexpr double kEpsilon = 0.1;

//去掉相同重复的点
void RemoveDuplicates(std::vector<Vec2d> *points) {
  RETURN_IF_NULL(points);

  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (const auto &point : *points) {
    if (count == 0 || point.DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = point;
    }
  }
  points->resize(count);
}

/*
lane {
  id {
    id: "lane_13"
  }
  central_curve {
    segment {
      line_segment {
        point {
          x: 587005.2303237915
          y: 4141592.8272733688
        }
        point {
          x: 587020.93030166626
          y: 4141588.5776176453
        }
        point {
          x: 587039.40564918518
          y: 4141582.90659523
        }
      }
      s: 0
      start_position {
        x: 587005.2303237915
        y: 4141592.8272733688
        z: 0
      }
      length: 35.591361999511719
    }
  }
  */
 //从地图中的central_curve->segment->line_segment中得到各个（地图原始的）point
void PointsFromCurve(const Curve &input_curve, std::vector<Vec2d> *points) {
  RETURN_IF_NULL(points);
  points->clear();

  for (const auto &curve : input_curve.segment()) {
    if (curve.has_line_segment()) {
      for (const auto &point : curve.line_segment().point()) {
        points->emplace_back(point.x(), point.y());
      }
    } else {
      AERROR << "Can not handle curve type.";
    }
  }
  RemoveDuplicates(points);
}

//被crosswalk等类型地图调用进行初始化，形成多边形Polygon2d
apollo::common::math::Polygon2d ConvertToPolygon2d(const Polygon &polygon) {
  std::vector<Vec2d> points;
  points.reserve(polygon.point_size());
  for (const auto &point : polygon.point()) {
    points.emplace_back(point.x(), point.y());
  }
  RemoveDuplicates(&points);

  //第一个点和最后一个点距离重复时，把最后一个点删除
  while (points.size() >= 2 && points[0].DistanceTo(points.back()) <=
                                   apollo::common::math::kMathEpsilon) {
    points.pop_back();
  }
  return apollo::common::math::Polygon2d(points);
}

//根据curve，得到points，进而得到线段vector（segments）
void SegmentsFromCurve(
    const Curve &curve,
    std::vector<apollo::common::math::LineSegment2d> *segments) {
  RETURN_IF_NULL(segments);

  std::vector<Vec2d> points;
  PointsFromCurve(curve, &points);
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    segments->emplace_back(points[i], points[i + 1]);
  }
}

//数据结构转换，由Vec2d转化为PointENU
PointENU PointFromVec2d(const Vec2d &point) {
  PointENU pt;
  pt.set_x(point.x());
  pt.set_y(point.y());
  return pt;
}

}  // namespace

LaneInfo::LaneInfo(const Lane &lane) : lane_(lane) { Init(); }

/*
lane {
  id {
    id: "lane_13"
  }
  central_curve {
    segment {
      line_segment {
        point {
          x: 587005.2303237915
          y: 4141592.8272733688
        }
        point {
          x: 587020.93030166626
          y: 4141588.5776176453
        }
        point {
          x: 587039.40564918518
          y: 4141582.90659523
        }
      }
      s: 0
      start_position {
        x: 587005.2303237915
        y: 4141592.8272733688
        z: 0
      }
      length: 35.591361999511719
    }
  }
  left_boundary {
    curve {
      segment {
        line_segment {
          point {
            x: 587005.68755340576
            y: 4141594.516456604
          }
          point {
            x: 587021.41816329956
            y: 4141590.2582302094
          }
          point {
            x: 587039.91916656494
            y: 4141584.5795555115
          }
        }
        s: 0
        start_position {
          x: 587005.68755340576
          y: 4141594.516456604
        }
        length: 35.649940490722656
      }
    }
    length: 35.649940490722656
    boundary_type {
      s: 0
      types: DOUBLE_YELLOW
    }
  }
  right_boundary {
    curve {
      segment {
        line_segment {
          point {
            x: 587004.77309417725
            y: 4141591.13808918
          }
          point {
            x: 587020.442440033
            y: 4141586.8970050812
          }
          point {
            x: 587038.89213180542
            y: 4141581.2336349487
          }
        }
        s: 0
        start_position {
          x: 587005.68755340576
          y: 4141594.516456604
        }
        length: 35.649940490722656
      }
    }
    length: 35.532783508300781
    boundary_type {
      types: DOTTED_WHITE
    }
  }
  length: 35.591361999511719
  speed_limit: 20.117000579833984
  successor_id {
    id: "lane_36"
  }
  right_neighbor_forward_lane_id {
    id: "lane_12"
  }
  type: CITY_DRIVING
  turn: NO_TURN
  left_sample {
    s: 0
    width: 1.75
  }
  left_sample {
    s: 16.265226364135742
    width: 1.75
  }
  left_sample {
    s: 35.591361999511719
    width: 1.75
  }
  right_sample {
    s: 0
    width: 1.75
  }
  right_sample {
    s: 16.265226364135742
    width: 1.75
  }
  right_sample {
    s: 35.591361999511719
    width: 1.75
  }
  direction: FORWARD
  left_road_sample {
    s: 0
    width: 1.75
  }
  left_road_sample {
    s: 16.265226364135742
    width: 1.75
  }
  left_road_sample {
    s: 35.591361999511719
    width: 1.75
  }
  right_road_sample {
    s: 0
    width: 1.75
  }
  right_road_sample {
    s: 16.265226364135742
    width: 1.75
  }
  right_road_sample {
    s: 35.591361999511719
    width: 1.75
  }
}
*/
//地图中的每一个lane进行初始化
void LaneInfo::Init() {
  PointsFromCurve(lane_.central_curve(), &points_);
  CHECK_GE(points_.size(), 2U);
  segments_.clear();
  accumulated_s_.clear();
  unit_directions_.clear();
  headings_.clear();

  double s = 0;
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    segments_.emplace_back(points_[i], points_[i + 1]);//连续两点间用直线描述
    accumulated_s_.push_back(s);
    unit_directions_.push_back(segments_.back().unit_direction());//计算直线段方向
    s += segments_.back().length();
  }

  accumulated_s_.push_back(s);//累加s
  total_length_ = s;//每条lane的起始s都是0
  ACHECK(!unit_directions_.empty());
  unit_directions_.push_back(unit_directions_.back());
  for (const auto &direction : unit_directions_) {
    headings_.push_back(direction.Angle());//根据方向计算角度
  }
  for (const auto &overlap_id : lane_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id.id());//重叠的id
  }
  ACHECK(!segments_.empty());

  sampled_left_width_.clear();
  sampled_right_width_.clear();
  for (const auto &sample : lane_.left_sample()) {
    sampled_left_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_sample()) {
    sampled_right_width_.emplace_back(sample.s(), sample.width());
  }

//如果是城市道路，检查左侧、右侧宽度是否＞半个车宽
  if (lane_.has_type()) {
    if (lane_.type() == Lane::CITY_DRIVING) {
      for (const auto &p : sampled_left_width_) {
        if (p.second < FLAGS_half_vehicle_width) {//默认1.05m
          AERROR
              << "lane[id = " << lane_.id().DebugString()
              << "]. sampled_left_width_[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << FLAGS_half_vehicle_width << "].";
        }
      }
      for (const auto &p : sampled_right_width_) {
        if (p.second < FLAGS_half_vehicle_width) {
          AERROR
              << "lane[id = " << lane_.id().DebugString()
              << "]. sampled_right_width_[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << FLAGS_half_vehicle_width << "].";
        }
      }
    } else if (lane_.type() == Lane::NONE) {
      AERROR << "lane_[id = " << lane_.id().DebugString() << "] type is NONE.";
    }
  } else {
    AERROR << "lane_[id = " << lane_.id().DebugString() << "] has NO type.";
  }

  sampled_left_road_width_.clear();
  sampled_right_road_width_.clear();
  for (const auto &sample : lane_.left_road_sample()) {
    sampled_left_road_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_road_sample()) {
    sampled_right_road_width_.emplace_back(sample.s(), sample.width());
  }

  CreateKDTree();
}


//根据s，分别查找s对应的左侧和右侧路宽
void LaneInfo::GetWidth(const double s, double *left_width,
                        double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_width_, s);
  }
}

//
double LaneInfo::Heading(const double s) const {
  const double kEpsilon = 0.001;
  if (s + kEpsilon < accumulated_s_.front()) {
    AERROR << "s:" << s << " should be >= " << accumulated_s_.front();
    return 0.0;
  }
  if (s - kEpsilon > accumulated_s_.back()) {
    AERROR << "s:" << s << " should be <= " << accumulated_s_.back();
    return 0.0;
  }

//根据accumulated_s_找到s对应的索引
  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0 || *iter - s <= common::math::kMathEpsilon) {
    return headings_[index];
  }
  return common::math::slerp(headings_[index - 1], accumulated_s_[index - 1],
                             headings_[index], accumulated_s_[index], s);
}

//根据s处的delts和deltheading计算曲率
double LaneInfo::Curvature(const double s) const {
  if (points_.size() < 2U) {
    AERROR << "Not enough points to compute curvature.";
    return 0.0;
  }
  const double kEpsilon = 0.001;
  if (s + kEpsilon < accumulated_s_.front()) {
    AERROR << "s:" << s << " should be >= " << accumulated_s_.front();
    return 0.0;
  }
  if (s > accumulated_s_.back() + kEpsilon) {
    AERROR << "s:" << s << " should be <= " << accumulated_s_.back();
    return 0.0;
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (iter == accumulated_s_.end()) {
    ADEBUG << "Reach the end of lane.";
    return 0.0;
  }
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0) {
    ADEBUG << "Reach the beginning of lane";
    return 0.0;
  }
  return (headings_[index] - headings_[index - 1]) /
         (accumulated_s_[index] - accumulated_s_[index - 1] + kEpsilon);
}

//根据s查询对应的车道宽度，左侧+右侧
double LaneInfo::GetWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return left_width + right_width;
}

//根据s查询对应的车道宽度，2倍的min(左侧、右侧)
double LaneInfo::GetEffectiveWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return 2 * std::min(left_width, right_width);
}

void LaneInfo::GetRoadWidth(const double s, double *left_width,
                            double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_road_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_road_width_, s);
  }
}

double LaneInfo::GetRoadWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetRoadWidth(s, &left_width, &right_width);
  return left_width + right_width;
}

//根据s和lane的采样路宽，查找线性差值得到s处的路宽
double LaneInfo::GetWidthFromSample(
    const std::vector<LaneInfo::SampledWidth> &samples, const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= samples[0].first) {
    return samples[0].second;
  }
  if (s >= samples.back().first) {
    return samples.back().second;
  }
  int low = 0;
  int high = static_cast<int>(samples.size());
  //通过二分法找到s对应的samples索引的low和high
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (samples[mid].first <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  const LaneInfo::SampledWidth &sample1 = samples[low];
  const LaneInfo::SampledWidth &sample2 = samples[high];
  //根据s、low、high对应的s进行线性差值，得到采样s对应的路宽
  const double ratio = (sample2.first - s) / (sample2.first - sample1.first);
  return sample1.second * ratio + sample2.second * (1.0 - ratio);
}

bool LaneInfo::IsOnLane(const Vec2d &point) const {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  if (!GetProjection(point, &accumulate_s, &lateral)) {
    return false;
  }

//如果s超出了lane的总长度，则返回错误
  if (accumulate_s > (total_length() + kEpsilon) ||
      (accumulate_s + kEpsilon) < 0.0) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  //如果l值在车道宽度内，则返回true
  GetWidth(accumulate_s, &left_width, &right_width);
  if (lateral < left_width && lateral > -right_width) {
    return true;
  }
  return false;
}

//判断输入的包围盒obb，是否在车道上（通过判断所有角点是否在车道内
bool LaneInfo::IsOnLane(const apollo::common::math::Box2d &box) const {
  std::vector<Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto &corner : corners) {
    if (!IsOnLane(corner)) {
      return false;
    }
  }
  return true;
}

//根据s值查找高精地图的关键点（ENU)，若s值与关键点s值差值小于阈值，直接返回，否则进行插值计算
PointENU LaneInfo::GetSmoothPoint(double s) const {
  PointENU point;
  RETURN_VAL_IF(points_.size() < 2, point);
  if (s <= 0.0) {
    return PointFromVec2d(points_[0]);
  }

  if (s >= total_length()) {
    return PointFromVec2d(points_.back());
  }

  const auto low_itr =
      std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  RETURN_VAL_IF(low_itr == accumulated_s_.end(), point);
  size_t index = low_itr - accumulated_s_.begin();
  double delta_s = *low_itr - s;
  if (delta_s < apollo::common::math::kMathEpsilon) {
    return PointFromVec2d(points_[index]);
  }

  auto smooth_point = points_[index] - unit_directions_[index - 1] * delta_s;

  return PointFromVec2d(smooth_point);
}

double LaneInfo::DistanceTo(const Vec2d &point) const {
  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  RETURN_VAL_IF_NULL(segment_box, 0.0);
  return segment_box->DistanceTo(point);
}

//求点到车道的最近距离及便宜s值和索引
double LaneInfo::DistanceTo(const Vec2d &point, Vec2d *map_point,
                            double *s_offset, int *s_offset_index) const {
  RETURN_VAL_IF_NULL(map_point, 0.0);
  RETURN_VAL_IF_NULL(s_offset, 0.0);
  RETURN_VAL_IF_NULL(s_offset_index, 0.0);

  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  RETURN_VAL_IF_NULL(segment_box, 0.0);
  int index = segment_box->id();
  double distance = segments_[index].DistanceTo(point, map_point);
  *s_offset_index = index;
  *s_offset =
      accumulated_s_[index] + segments_[index].start().DistanceTo(*map_point);
  return distance;
}

PointENU LaneInfo::GetNearestPoint(const Vec2d &point, double *distance) const {
  PointENU empty_point;
  RETURN_VAL_IF_NULL(distance, empty_point);

  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  RETURN_VAL_IF_NULL(segment_box, empty_point);
  int index = segment_box->id();
  Vec2d nearest_point;
  *distance = segments_[index].DistanceTo(point, &nearest_point);

  return PointFromVec2d(nearest_point);
}

//根据点xy坐标求s和l
bool LaneInfo::GetProjection(const Vec2d &point, double *accumulate_s,
                             double *lateral) const {
  RETURN_VAL_IF_NULL(accumulate_s, false);
  RETURN_VAL_IF_NULL(lateral, false);

  if (segments_.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  int seg_num = static_cast<int>(segments_.size());
  int min_index = 0;
  for (int i = 0; i < seg_num; ++i) {//求点到各个线段的垂线距离，找出最短距离对应的线段
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < min_dist) {
      min_index = i;
      min_dist = distance;
    }
  }
  min_dist = std::sqrt(min_dist);
  const auto &nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);//叉乘，垂线长度，左右是有符号的
  const auto proj = nearest_seg.ProjectOntoUnit(point);//内积，投影长度
  if (min_index == 0) {//说明垂点在第一个线段之外
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else if (min_index == seg_num - 1) {//说明垂点在最后一个线段之外
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else {//垂点在两个点之间
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
  }
  return true;
}

//地图数据后处理，更新overlap间的关系
void LaneInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

//地图数据后处理，更新overlap间的关系
void LaneInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr =
        map_instance.GetOverlapById(MakeMapId(overlap_id));//通过overlap_table_查询
    if (overlap_ptr == nullptr) {
      continue;
    }
    overlaps_.emplace_back(overlap_ptr);
    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == lane_.id().id()) {
        continue;
      }
      const auto &object_map_id = MakeMapId(object_id);
      if (map_instance.GetLaneById(object_map_id) != nullptr) {
        cross_lanes_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSignalById(object_map_id) != nullptr) {
        signals_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetYieldSignById(object_map_id) != nullptr) {
        yield_signs_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetStopSignById(object_map_id) != nullptr) {
        stop_signs_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetCrosswalkById(object_map_id) != nullptr) {
        crosswalks_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetJunctionById(object_map_id) != nullptr) {
        junctions_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetClearAreaById(object_map_id) != nullptr) {
        clear_areas_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSpeedBumpById(object_map_id) != nullptr) {
        speed_bumps_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetParkingSpaceById(object_map_id) != nullptr) {
        parking_spaces_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetPNCJunctionById(object_map_id) != nullptr) {
        pnc_junctions_.emplace_back(overlap_ptr);
      }
    }
  }
}

void LaneInfo::CreateKDTree() {
  apollo::common::math::AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;

  segment_box_list_.clear();
  for (size_t id = 0; id < segments_.size(); ++id) {//segments是地图中点-点依次组成的线段
    const auto &segment = segments_[id];
    segment_box_list_.emplace_back(
        apollo::common::math::AABox2d(segment.start(), segment.end()), this,
        &segment, id);
  }
  lane_segment_kdtree_.reset(new LaneSegmentKDTree(segment_box_list_, params));
}

JunctionInfo::JunctionInfo(const Junction &junction) : junction_(junction) {
  Init();
}

void JunctionInfo::Init() {
  polygon_ = ConvertToPolygon2d(junction_.polygon());
  CHECK_GT(polygon_.num_points(), 2);

  for (const auto &overlap_id : junction_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

void JunctionInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

void JunctionInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      continue;
    }

    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == id().id()) {
        continue;
      }

      if (object.has_stop_sign_overlap_info()) {
        overlap_stop_sign_ids_.push_back(object.id());
      }
    }
  }
}

SignalInfo::SignalInfo(const Signal &signal) : signal_(signal) { Init(); }

void SignalInfo::Init() {
  for (const auto &stop_line : signal_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  ACHECK(!segments_.empty());
  std::vector<Vec2d> points;
  for (const auto &segment : segments_) {
    points.emplace_back(segment.start());
    points.emplace_back(segment.end());
  }
  CHECK_GT(points.size(), 0U);
}

CrosswalkInfo::CrosswalkInfo(const Crosswalk &crosswalk)
    : crosswalk_(crosswalk) {
  Init();
}

void CrosswalkInfo::Init() {
  polygon_ = ConvertToPolygon2d(crosswalk_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}

StopSignInfo::StopSignInfo(const StopSign &stop_sign) : stop_sign_(stop_sign) {
  init();
}

void StopSignInfo::init() {
  for (const auto &stop_line : stop_sign_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  ACHECK(!segments_.empty());

  for (const auto &overlap_id : stop_sign_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

void StopSignInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

void StopSignInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);//遍历overlap_tables
    if (overlap_ptr == nullptr) {
      continue;
    }

    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == id().id()) {
        continue;
      }

      if (object.has_junction_overlap_info()) {
        overlap_junction_ids_.push_back(object.id());
      } else if (object.has_lane_overlap_info()) {
        overlap_lane_ids_.push_back(object.id());
      }
    }
  }
  if (overlap_junction_ids_.empty()) {
    AWARN << "stop sign " << id().id() << "has no overlap with any junction.";
  }
}

YieldSignInfo::YieldSignInfo(const YieldSign &yield_sign)
    : yield_sign_(yield_sign) {
  Init();
}

void YieldSignInfo::Init() {
  for (const auto &stop_line : yield_sign_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  // segments_from_curve(yield_sign_.stop_line(), &segments_);
  ACHECK(!segments_.empty());
}

ClearAreaInfo::ClearAreaInfo(const ClearArea &clear_area)
    : clear_area_(clear_area) {
  Init();
}

void ClearAreaInfo::Init() {
  polygon_ = ConvertToPolygon2d(clear_area_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}

SpeedBumpInfo::SpeedBumpInfo(const SpeedBump &speed_bump)
    : speed_bump_(speed_bump) {
  Init();
}

void SpeedBumpInfo::Init() {
  for (const auto &stop_line : speed_bump_.position()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  ACHECK(!segments_.empty());
}

OverlapInfo::OverlapInfo(const Overlap &overlap) : overlap_(overlap) {}

const ObjectOverlapInfo *OverlapInfo::GetObjectOverlapInfo(const Id &id) const {
  for (const auto &object : overlap_.object()) {
    if (object.id().id() == id.id()) {
      return &object;
    }
  }
  return nullptr;
}

RoadInfo::RoadInfo(const Road &road) : road_(road) {
  for (const auto &section : road_.section()) {
    sections_.push_back(section);
    road_boundaries_.push_back(section.boundary());
  }
}

const std::vector<RoadBoundary> &RoadInfo::GetBoundaries() const {
  return road_boundaries_;
}

ParkingSpaceInfo::ParkingSpaceInfo(const ParkingSpace &parking_space)
    : parking_space_(parking_space) {
  Init();
}

void ParkingSpaceInfo::Init() {
  polygon_ = ConvertToPolygon2d(parking_space_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}

PNCJunctionInfo::PNCJunctionInfo(const PNCJunction &pnc_junction)
    : junction_(pnc_junction) {
  Init();
}

void PNCJunctionInfo::Init() {
  polygon_ = ConvertToPolygon2d(junction_.polygon());
  CHECK_GT(polygon_.num_points(), 2);

  for (const auto &overlap_id : junction_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

RSUInfo::RSUInfo(const RSU &rsu) : _rsu(rsu) {}

}  // namespace hdmap
}  // namespace apollo

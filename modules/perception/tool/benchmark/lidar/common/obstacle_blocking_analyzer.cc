/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/common/obstacle_blocking_analyzer.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;

constexpr double kAdcDistanceThreshold = 35.0;  // unit: m
constexpr double kObstaclesDistanceThreshold = 15.0;

bool IsNonmovableObstacle(const ReferenceLineInfo& reference_line_info,
                          const Obstacle& obstacle) {
  // Obstacle is far away.
  //障碍物起点s距离车头超出35m，认为太远，不确定障碍物状态，不允许借道
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  if (obstacle.PerceptionSLBoundary().start_s() >
      adc_sl_boundary.end_s() + kAdcDistanceThreshold) {
    ADEBUG << " - It is too far ahead and we are not so sure of its status.";
    return false;
  }

  // Obstacle is parked obstacle.
  if (IsParkedVehicle(reference_line_info.reference_line(), &obstacle)) {
    ADEBUG << "It is Parked and NON-MOVABLE.";
    return true;
  }

  // Obstacle is blocked by others too.
  //判断该障碍物是否被其他障碍物阻塞
  for (const auto* other_obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (other_obstacle->Id() == obstacle.Id()) {
      continue;
    }
    if (other_obstacle->IsVirtual()) {
      continue;
    }
    //判断该障碍物与其他障碍物在横向是否有重叠
    if (other_obstacle->PerceptionSLBoundary().start_l() >
            obstacle.PerceptionSLBoundary().end_l() ||
        other_obstacle->PerceptionSLBoundary().end_l() <
            obstacle.PerceptionSLBoundary().start_l()) {
      // not blocking the backside vehicle
      continue;
    }
    //若横向有重叠，判断其他障碍物的起始s值-该障碍物的终止s值
    //如果差值小于0，说明有重叠，或者>阈值，继续循环
    double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                     obstacle.PerceptionSLBoundary().end_s();
    if (delta_s < 0.0 || delta_s > kObstaclesDistanceThreshold) {
      continue;
    }
    //如果上述差值在0和阈值范围内，认为该障碍物是被其他障碍物阻塞，将会移动
    // TODO(All): Fix the segmentation bug for large vehicles, otherwise
    // the follow line will be problematic.
    ADEBUG << " - It is blocked by others, and will move later.";
    return false;
  }

  ADEBUG << "IT IS NON-MOVABLE!";
  return true;
}

// This is the side-pass condition for every obstacle.
// TODO(all): if possible, transform as many function parameters into GFLAGS.
bool IsBlockingObstacleToSidePass(const Frame& frame, const Obstacle* obstacle,
                                  double block_obstacle_min_speed,
                                  double min_front_sidepass_distance,
                                  bool enable_obstacle_blocked_check) {
  // Get the necessary info.
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const PathDecision& path_decision = reference_line_info.path_decision();
  ADEBUG << "Evaluating Obstacle: " << obstacle->Id();

  // Obstacle is virtual.
  if (obstacle->IsVirtual()) {
    ADEBUG << " - It is virtual.";
    return false;
  }

  // Obstacle is moving.
  if (!obstacle->IsStatic() || obstacle->speed() > block_obstacle_min_speed) {
    ADEBUG << " - It is non-static.";
    return false;
  }

  // Obstacle is behind ADC.
  if (obstacle->PerceptionSLBoundary().start_s() <= adc_sl_boundary.end_s()) {
    ADEBUG << " - It is behind ADC.";
    return false;
  }

  // Obstacle is far away.
  static constexpr double kAdcDistanceSidePassThreshold = 15.0;
  if (obstacle->PerceptionSLBoundary().start_s() >
      adc_sl_boundary.end_s() + kAdcDistanceSidePassThreshold) {
    ADEBUG << " - It is too far ahead.";
    return false;
  }

  // Obstacle is too close.
  if (adc_sl_boundary.end_s() + min_front_sidepass_distance >
      obstacle->PerceptionSLBoundary().start_s()) {
    ADEBUG << " - It is too close to side-pass.";
    return false;
  }

  // Obstacle is not blocking our path.
  if (!IsBlockingDrivingPathObstacle(reference_line, obstacle)) {
    ADEBUG << " - It is not blocking our way.";
    return false;
  }

  // Obstacle is blocked by others too.
  if (enable_obstacle_blocked_check &&
      !IsParkedVehicle(reference_line, obstacle)) {
    for (const auto* other_obstacle : path_decision.obstacles().Items()) {
      if (other_obstacle->Id() == obstacle->Id()) {
        continue;
      }
      if (other_obstacle->IsVirtual()) {
        continue;
      }
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              obstacle->PerceptionSLBoundary().end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              obstacle->PerceptionSLBoundary().start_l()) {
        // not blocking the backside vehicle
        continue;
      }
      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                       obstacle->PerceptionSLBoundary().end_s();
      if (delta_s < 0.0 || delta_s > kAdcDistanceThreshold) {
        continue;
      }

      // TODO(All): Fix the segmentation bug for large vehicles, otherwise
      // the follow line will be problematic.
      ADEBUG << " - It is blocked by others, too.";
      return false;
    }
  }

  ADEBUG << "IT IS BLOCKING!";
  return true;
}

double GetDistanceBetweenADCAndObstacle(const Frame& frame,
                                        const Obstacle* obstacle) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  double distance_between_adc_and_obstacle =
      obstacle->PerceptionSLBoundary().start_s() - adc_sl_boundary.end_s();
  return distance_between_adc_and_obstacle;
}

bool IsBlockingDrivingPathObstacle(const ReferenceLine& reference_line,
                                   const Obstacle* obstacle) {
  const double driving_width =
      reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());
  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  ADEBUG << " (driving width = " << driving_width
         << ", adc_width = " << adc_width << ")";
  if (driving_width > adc_width + FLAGS_static_obstacle_nudge_l_buffer +
                          FLAGS_side_pass_driving_width_l_buffer) {
    // TODO(jiacheng): make this a GFLAG:
    // side_pass_context_.scenario_config_.min_l_nudge_buffer()
    ADEBUG << "It is NOT blocking our path.";
    return false;
  }

  ADEBUG << "It is blocking our path.";
  return true;
}

bool IsParkedVehicle(const ReferenceLine& reference_line,
                     const Obstacle* obstacle) {
  if (!FLAGS_enable_scenario_side_pass_multiple_parked_obstacles) {
    return false;
  }
  double road_left_width = 0.0;
  double road_right_width = 0.0;
  double max_road_right_width = 0.0;
  //获取车头、车尾处的车道宽度，并取最大值，若障碍物起始l值>车道宽度-0.1,则认为是在车道边缘（该函数调用前提是，障碍物是阻塞）
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().start_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = road_right_width;
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().end_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = std::max(max_road_right_width, road_right_width);
  bool is_at_road_edge = std::abs(obstacle->PerceptionSLBoundary().start_l()) >
                         max_road_right_width - 0.1;

  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  auto obstacle_box = obstacle->PerceptionBoundingBox();
  HDMapUtil::BaseMapPtr()->GetLanes(
      common::util::PointFactory::ToPointENU(obstacle_box.center().x(),
                                             obstacle_box.center().y()),
      std::min(obstacle_box.width(), obstacle_box.length()), &lanes);
  bool is_on_parking_lane = false;
  //如果只有一个车道，且车道类型为parking,标志位置true
  if (lanes.size() == 1 &&
      lanes.front()->lane().type() == apollo::hdmap::Lane::PARKING) {
    is_on_parking_lane = true;
  }
//如果是在停车车道上或者是在车道边缘，且障碍物是静态，则返回true
  bool is_parked = is_on_parking_lane || is_at_road_edge;
  return is_parked && obstacle->IsStatic();
}

}  // namespace planning
}  // namespace apollo

/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

#include <limits>
#include <memory>

#include "cyber/time/clock.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::cyber::Clock;

LaneChangeDecider::LaneChangeDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  ACHECK(config_.has_lane_change_decider_config());
}

// added a dummy parameter to enable this task in ExecuteTaskOnReferenceLine
Status LaneChangeDecider::Process(
    Frame* frame, ReferenceLineInfo* const current_reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);

  const auto& lane_change_decider_config = config_.lane_change_decider_config();

  std::list<ReferenceLineInfo>* reference_line_info =
      frame->mutable_reference_line_info();
  if (reference_line_info->empty()) {
    const std::string msg = "Reference lines empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

//根据是否优先换道，参考线列表优先级排列
  if (lane_change_decider_config.reckless_change_lane()) {
    PrioritizeChangeLane(true, reference_line_info);
    return Status::OK();
  }

//获取上一时刻车辆变道的状态
  auto* prev_status = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_change_lane();
  double now = Clock::NowInSeconds();

//如果当前参考线为变道，判断当前参考线是否有障碍，并对状态进行赋值
  prev_status->set_is_clear_to_change_lane(false);
  if (current_reference_line_info->IsChangeLanePath()) {
    prev_status->set_is_clear_to_change_lane(
        IsClearToChangeLane(current_reference_line_info));
  }

/*  enum Status {
    IN_CHANGE_LANE = 1;        // during change lane state
    CHANGE_LANE_FAILED = 2;    // change lane failed
    CHANGE_LANE_FINISHED = 3;  // change lane finished
  }
  */
 //如果获取不到上一时刻变道状态，更新状态为变道成功
  if (!prev_status->has_status()) {
    UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                 GetCurrentPathId(*reference_line_info));
    prev_status->set_last_succeed_timestamp(now);
    return Status::OK();
  }
//参考线超过一条，说明有变道参考线
  bool has_change_lane = reference_line_info->size() > 1;
  ADEBUG << "has_change_lane: " << has_change_lane;
  //如果只有一条参考线，
  //上一时刻状态为变道成功，状态保持
  //上一时刻状态是变道中，则状态更新为变道成功,
  //上一时刻状态为变道失败，状态保持
  if (!has_change_lane) {
    const auto& path_id = reference_line_info->front().Lanes().Id();
    if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FINISHED) {
    } else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, path_id);
    } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
    } else {
      const std::string msg =
          absl::StrCat("Unknown state: ", prev_status->ShortDebugString());
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    return Status::OK();
  } else {  // has change lane in reference lines.有多条参考线
    auto current_path_id = GetCurrentPathId(*reference_line_info);
    if (current_path_id.empty()) {
      const std::string msg = "The vehicle is not on any reference line";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    //如果上一时刻状态为变道中，若上一时刻道路id和当前道路id相同，则状态保持，返回（把变道参考线优先级提高）
    //如果上一时刻道路id和当前道路id不相同，则把非变道参考线优先级提高
    if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      if (prev_status->path_id() == current_path_id) {
        PrioritizeChangeLane(true, reference_line_info);
      } else {
        // RemoveChangeLane(reference_line_info);
        PrioritizeChangeLane(false, reference_line_info);
        ADEBUG << "removed change lane.";
        UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                     current_path_id);
      }
      return Status::OK();
    } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
      // TODO(SHU): add an optimization_failure counter to enter
      // change_lane_failed status
      //如果变道失败，如果与上一时刻时间差小于设定阈值，则把非变道参考线优先级提高
                   //否则，更新状态为变道中
      if (now - prev_status->timestamp() <
          lane_change_decider_config.change_lane_fail_freeze_time()) {
        // RemoveChangeLane(reference_line_info);
        PrioritizeChangeLane(false, reference_line_info);
        ADEBUG << "freezed after failed";
      } else {
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
        ADEBUG << "change lane again after failed";
      }
      return Status::OK();
    //如果变道成功，如果与上一时刻时间差小于设定阈值，则把非变道参考线优先级提高
                 //否则，更新状态为变道中，把变道参考线优先级提高
    } else if (prev_status->status() ==
               ChangeLaneStatus::CHANGE_LANE_FINISHED) {
      if (now - prev_status->timestamp() <
          lane_change_decider_config.change_lane_success_freeze_time()) {
        // RemoveChangeLane(reference_line_info);
        PrioritizeChangeLane(false, reference_line_info);
        ADEBUG << "freezed after completed lane change";
      } else {
        PrioritizeChangeLane(true, reference_line_info);
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
        ADEBUG << "change lane again after success";
      }
    } else {
      const std::string msg =
          absl::StrCat("Unknown state: ", prev_status->ShortDebugString());
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

void LaneChangeDecider::UpdatePreparationDistance(
    const bool is_opt_succeed, const Frame* frame,
    const ReferenceLineInfo* const reference_line_info,
    PlanningContext* planning_context) {
  auto* lane_change_status =
      planning_context->mutable_planning_status()->mutable_change_lane();
  ADEBUG << "Current time: " << lane_change_status->timestamp();
  ADEBUG << "Lane Change Status: " << lane_change_status->status();
  // If lane change planning succeeded, update and return
  if (is_opt_succeed) {
    lane_change_status->set_last_succeed_timestamp(
        Clock::NowInSeconds());
    lane_change_status->set_is_current_opt_succeed(true);
    return;
  }
  // If path optimizer or speed optimizer failed, report the status
  lane_change_status->set_is_current_opt_succeed(false);
  // If the planner just succeed recently, let's be more patient and try again
  if (Clock::NowInSeconds() -
          lane_change_status->last_succeed_timestamp() <
      FLAGS_allowed_lane_change_failure_time) {
    return;
  }
  // Get ADC's current s and the lane-change start distance s
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  const common::TrajectoryPoint& planning_start_point =
      frame->PlanningStartPoint();
  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  if (!lane_change_status->exist_lane_change_start_position()) {
    return;
  }
  common::SLPoint point_sl;
  reference_line.XYToSL(lane_change_status->lane_change_start_position(),
                        &point_sl);
  ADEBUG << "Current ADC s: " << adc_sl_info.first[0];
  ADEBUG << "Change lane point s: " << point_sl.s();
  // If the remaining lane-change preparation distance is too small,
  // refresh the preparation distance
  if (adc_sl_info.first[0] + FLAGS_min_lane_change_prepare_length >
      point_sl.s()) {
    lane_change_status->set_exist_lane_change_start_position(false);
    ADEBUG << "Refresh the lane-change preparation distance";
  }
}

void LaneChangeDecider::UpdateStatus(ChangeLaneStatus::Status status_code,
                                     const std::string& path_id) {
  UpdateStatus(Clock::NowInSeconds(), status_code, path_id);
}
//换道状态更新，（时间，道路id，换道状态）
void LaneChangeDecider::UpdateStatus(double timestamp,
                                     ChangeLaneStatus::Status status_code,
                                     const std::string& path_id) {
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);
}

//若配置文件使能了换道优先，根据形参，若换道优先把换道放到参考线列表最前面，否则把非换道放到参考线列表最前面
void LaneChangeDecider::PrioritizeChangeLane(
    const bool is_prioritize_change_lane,
    std::list<ReferenceLineInfo>* reference_line_info) const {
  if (reference_line_info->empty()) {
    AERROR << "Reference line info empty";
    return;
  }

  const auto& lane_change_decider_config = config_.lane_change_decider_config();

  // TODO(SHU): disable the reference line order change for now
  if (!lane_change_decider_config.enable_prioritize_change_lane()) {
    return;
  }
  auto iter = reference_line_info->begin();
  while (iter != reference_line_info->end()) {
    ADEBUG << "iter->IsChangeLanePath(): " << iter->IsChangeLanePath();
    /* is_prioritize_change_lane == true: prioritize change_lane_reference_line
       is_prioritize_change_lane == false: prioritize
       non_change_lane_reference_line */
    if ((is_prioritize_change_lane && iter->IsChangeLanePath()) ||
        (!is_prioritize_change_lane && !iter->IsChangeLanePath())) {
      ADEBUG << "is_prioritize_change_lane: " << is_prioritize_change_lane;
      ADEBUG << "iter->IsChangeLanePath(): " << iter->IsChangeLanePath();
      break;
    }
    ++iter;
  }
  reference_line_info->splice(reference_line_info->begin(),
                              *reference_line_info, iter);
  ADEBUG << "reference_line_info->IsChangeLanePath(): "
         << reference_line_info->begin()->IsChangeLanePath();
}

// disabled for now
void LaneChangeDecider::RemoveChangeLane(
    std::list<ReferenceLineInfo>* reference_line_info) const {
  const auto& lane_change_decider_config = config_.lane_change_decider_config();
  // TODO(SHU): fix core dump when removing change lane
  if (!lane_change_decider_config.enable_remove_change_lane()) {
    return;
  }
  ADEBUG << "removed change lane";
  auto iter = reference_line_info->begin();
  while (iter != reference_line_info->end()) {
    if (iter->IsChangeLanePath()) {
      iter = reference_line_info->erase(iter);
    } else {
      ++iter;
    }
  }
}

std::string LaneChangeDecider::GetCurrentPathId(
    const std::list<ReferenceLineInfo>& reference_line_info) const {
  for (const auto& info : reference_line_info) {
    if (!info.IsChangeLanePath()) {
      return info.Lanes().Id();
    }
  }
  return "";
}

//被障碍物阻塞，返回false（不关心虚拟和静态障碍物)
//无阻碍，返回true
bool LaneChangeDecider::IsClearToChangeLane(
    ReferenceLineInfo* reference_line_info) {
  double ego_start_s = reference_line_info->AdcSlBoundary().start_s();
  double ego_end_s = reference_line_info->AdcSlBoundary().end_s();
  double ego_v =
      std::abs(reference_line_info->vehicle_state().linear_velocity());

//障碍物遍历
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    //如果障碍物为虚拟或静态，则停止往下执行，进行下一个循环
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      ADEBUG << "skip one virtual or static obstacle";
      continue;
    }

    double start_s = std::numeric_limits<double>::max();
    double end_s = -std::numeric_limits<double>::max();
    double start_l = std::numeric_limits<double>::max();
    double end_l = -std::numeric_limits<double>::max();
    //遍历该障碍物的边角点，找出最小、最大的s，l
    for (const auto& p : obstacle->PerceptionPolygon().points()) {
      SLPoint sl_point;
      //障碍物边角点由x,y坐标转化为sl
      reference_line_info->reference_line().XYToSL(p, &sl_point);
      start_s = std::fmin(start_s, sl_point.s());
      end_s = std::fmax(end_s, sl_point.s());

      start_l = std::fmin(start_l, sl_point.l());
      end_l = std::fmax(end_l, sl_point.l());
    }
    //如果参考线为变道参考线，并且障碍物没有在参考线车道内，则停止向下执行，继续遍历下一个障碍物
    if (reference_line_info->IsChangeLanePath()) {
      static constexpr double kLateralShift = 2.5;
      if (end_l < -kLateralShift || start_l > kLateralShift) {
        continue;
      }
    }

    // Raw estimation on whether same direction with ADC or not based on
    // prediction trajectory
    bool same_direction = true;
    //如果障碍物有预测轨迹点序列，计算车辆航向和障碍物航向差值，小于90度，则认为航向一致
    if (obstacle->HasTrajectory()) {
      double obstacle_moving_direction =
          obstacle->Trajectory().trajectory_point(0).path_point().theta();
      const auto& vehicle_state = reference_line_info->vehicle_state();
      double vehicle_moving_direction = vehicle_state.heading();
      if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        vehicle_moving_direction =
            common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
      }
      double heading_difference = std::abs(common::math::NormalizeAngle(
          obstacle_moving_direction - vehicle_moving_direction));
      same_direction = heading_difference < (M_PI / 2.0);
    }

    // TODO(All) move to confs
    static constexpr double kSafeTimeOnSameDirection = 3.0;
    static constexpr double kSafeTimeOnOppositeDirection = 5.0;
    static constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kForwardMinSafeDistanceOnOppositeDirection = 50.0;
    static constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
    static constexpr double kDistanceBuffer = 0.5;

    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;
    //障碍物与车运动方向相同，行驶安全间距会比运动方向相反的时候小一些
    if (same_direction) {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnSameDirection,
                    (ego_v - obstacle->speed()) * kSafeTimeOnSameDirection);
      kBackwardSafeDistance =
          std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                    (obstacle->speed() - ego_v) * kSafeTimeOnSameDirection);
    } else {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                    (ego_v + obstacle->speed()) * kSafeTimeOnOppositeDirection);
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }


    //若障碍物阻碍，则距离<安全间距+死区时，障碍物继续保持阻碍,>安全间距+死区时，才置位FALSE（无阻碍）
    //若障碍物无阻碍，则距离<安全间距-死区时，障碍物才置位true(阻碍)，>安全间距-死区时，障碍物保持无阻碍
    if (HysteresisFilter(ego_start_s - end_s, kBackwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
        HysteresisFilter(start_s - ego_end_s, kForwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(true);
      ADEBUG << "Lane Change is blocked by obstacle" << obstacle->Id();
      return false;
    } else {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(false);
    }
  }
  return true;
}

//基于光束扫描算法，障碍物阻塞，返回true
bool LaneChangeDecider::IsPerceptionBlocked(
    const ReferenceLineInfo& reference_line_info,
    const double search_beam_length, const double search_beam_radius_intensity,
    const double search_range, const double is_block_angle_threshold) {
  const auto& vehicle_state = reference_line_info.vehicle_state();
  const common::math::Vec2d adv_pos(vehicle_state.x(), vehicle_state.y());
  const double adv_heading = vehicle_state.heading();

//左大右小
//计算车头方向左侧最大扫描角度，右侧最大扫描角度
  double left_most_angle =
      common::math::NormalizeAngle(adv_heading + 0.5 * search_range);
  double right_most_angle =
      common::math::NormalizeAngle(adv_heading - 0.5 * search_range);
  bool right_most_found = false;

//遍历每一个障碍物
  for (auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    //如果障碍物为虚拟，进行下一个循环
    if (obstacle->IsVirtual()) {
      ADEBUG << "skip one virtual obstacle";
      continue;
    }
    const auto& obstacle_polygon = obstacle->PerceptionPolygon();
    //扫描角度从0递增
    for (double search_angle = 0.0; search_angle < search_range;
         search_angle += search_beam_radius_intensity) {
      common::math::Vec2d search_beam_end(search_beam_length, 0.0);
      //从右侧最小角度，开始递增计算光束角度
      const double beam_heading = common::math::NormalizeAngle(
          adv_heading - 0.5 * search_range + search_angle);
      search_beam_end.SelfRotate(beam_heading);//根据光束角计算旋转后向量
      search_beam_end += adv_pos;//计算光束端点，xy坐标
      common::math::LineSegment2d search_beam(adv_pos, search_beam_end);//计算光束端点-车身的线段

      //若上述线段与障碍物边框有重叠，则找到右侧角度
      if (!right_most_found && obstacle_polygon.HasOverlap(search_beam)) {
        right_most_found = true;
        right_most_angle = beam_heading;
      }

      //找到右侧重叠角度后，继续向左扫描到无重叠后，则是找到了左侧角度，退出扫描循环
      if (right_most_found && !obstacle_polygon.HasOverlap(search_beam)) {
        left_most_angle = beam_heading - search_angle;
        break;
      }
    }
    if (!right_most_found) {//上述扫描没有重叠，则障碍物未在扫描范围内
      // obstacle is not in search range
      continue;
    }
    if (std::fabs(common::math::NormalizeAngle(
            left_most_angle - right_most_angle)) > is_block_angle_threshold) {
      return true;
    }
  }

  return false;
}


bool LaneChangeDecider::HysteresisFilter(const double obstacle_distance,
                                         const double safe_distance,
                                         const double distance_buffer,
                                         const bool is_obstacle_blocking) {
  if (is_obstacle_blocking) {
    return obstacle_distance < safe_distance + distance_buffer;
  } else {
    return obstacle_distance < safe_distance - distance_buffer;
  }
}

}  // namespace planning
}  // namespace apollo

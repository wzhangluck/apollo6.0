/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/traffic_rules/crosswalk.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "cyber/time/clock.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/proto/planning_status.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;
using apollo::perception::PerceptionObstacle;
using CrosswalkToStop =
    std::vector<std::pair<const hdmap::PathOverlap*, std::vector<std::string>>>;
using CrosswalkStopTimer =
    std::unordered_map<std::string, std::unordered_map<std::string, double>>;

Crosswalk::Crosswalk(const TrafficRuleConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector)
    : TrafficRule(config, injector) {}

Status Crosswalk::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FindCrosswalks(reference_line_info)) {
    injector_->planning_context()->mutable_planning_status()->clear_crosswalk();
    return Status::OK();
  }

  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void Crosswalk::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  auto* mutable_crosswalk_status = injector_->planning_context()
                                       ->mutable_planning_status()
                                       ->mutable_crosswalk();

  auto* path_decision = reference_line_info->path_decision();
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  CrosswalkToStop crosswalks_to_stop;

  // read crosswalk_stop_timer from saved status
  CrosswalkStopTimer crosswalk_stop_timer;
  std::unordered_map<std::string, double> stop_times;
  for (const auto& stop_time : mutable_crosswalk_status->stop_time()) {
    stop_times.emplace(stop_time.obstacle_id(), stop_time.stop_timestamp_sec());
  }
  crosswalk_stop_timer.emplace(mutable_crosswalk_status->crosswalk_id(),
                               stop_times);

  const auto& finished_crosswalks =
      mutable_crosswalk_status->finished_crosswalk();

  const auto& reference_line = reference_line_info->reference_line();
  for (auto crosswalk_overlap : crosswalk_overlaps_) {
    auto crosswalk_ptr = HDMapUtil::BaseMap().GetCrosswalkById(
        hdmap::MakeMapId(crosswalk_overlap->object_id));
    std::string crosswalk_id = crosswalk_ptr->id().id();

    // skip crosswalk if master vehicle body already passes the stop line
    //车头驶过人行横道的距离超过阈值min_pass_s_distance时，清除rosswalk_status中的ID及停止时间信息,即不需要停车
    if (adc_front_edge_s - crosswalk_overlap->end_s >
        config_.crosswalk().min_pass_s_distance()) {//默认1m
      if (mutable_crosswalk_status->has_crosswalk_id() &&
          mutable_crosswalk_status->crosswalk_id() == crosswalk_id) {
        mutable_crosswalk_status->clear_crosswalk_id();
        mutable_crosswalk_status->clear_stop_time();
      }

      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id
             << "] crosswalk_overlap_end_s[" << crosswalk_overlap->end_s
             << "] adc_front_edge_s[" << adc_front_edge_s
             << "]. adc_front_edge passes crosswalk_end_s + buffer.";
      continue;
    }

    // check if crosswalk already finished
    //如果已经通过行人横道,则忽略,如果crosswalk_id已经在finished_crosswalks的里面了
    if (finished_crosswalks.end() != std::find(finished_crosswalks.begin(),
                                               finished_crosswalks.end(),
                                               crosswalk_id)) {
      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id << "] crosswalk_end_s["
             << crosswalk_overlap->end_s << "] finished already";
      continue;
    }

    std::vector<std::string> pedestrians;
    for (const auto* obstacle : path_decision->obstacles().Items()) {
      //根据车头位置和停止线之间的距离计算停车减速度;
      const double stop_deceleration = util::GetADCStopDeceleration(
          injector_->vehicle_state(), adc_front_edge_s,
          crosswalk_overlap->start_s);
      //判断正在遍历的障碍物是否需要停车
      bool stop = CheckStopForObstacle(reference_line_info, crosswalk_ptr,
                                       *obstacle, stop_deceleration);

      const std::string& obstacle_id = obstacle->Id();
      const PerceptionObstacle& perception_obstacle = obstacle->Perception();
      PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);

      // update stop timestamp on static pedestrian for watch timer
      const bool is_on_lane =
          reference_line.IsOnLane(obstacle->PerceptionSLBoundary());
      const double kStartWatchTimerDistance = 40.0;
      //如果需要停车,同时障碍物不在当前所查询的参考线所对应的车道上,且车辆到停止线的距离小于40米
      if (stop && !is_on_lane &&
          crosswalk_overlap->start_s - adc_front_edge_s <=
              kStartWatchTimerDistance) {
        // check on stop timer for static pedestrians/bicycles
        // if NOT on_lane ahead of adc
        const double kMaxStopSpeed = 0.3;
        auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                         perception_obstacle.velocity().y());
        if (obstacle_speed <= kMaxStopSpeed) {//当障碍物移动的速度小于等于0.3时
          if (crosswalk_stop_timer[crosswalk_id].count(obstacle_id) < 1) {
            //当前障碍物不在crosswalk_stop_timer[crosswalk_id]中,则将其插入其中
            // add timestamp
            ADEBUG << "add timestamp: obstacle_id[" << obstacle_id
                   << "] timestamp[" << Clock::NowInSeconds()
                   << "]";
            crosswalk_stop_timer[crosswalk_id].insert(
                {obstacle_id, Clock::NowInSeconds()});
          } else {
            //如果障碍物已经在了并且stop_time >= config_.crosswalk().stop_timeout(),则不再需要停车;
            double stop_time = Clock::NowInSeconds() -
                               crosswalk_stop_timer[crosswalk_id][obstacle_id];
            ADEBUG << "stop_time: obstacle_id[" << obstacle_id << "] stop_time["
                   << stop_time << "]";
            if (stop_time >= config_.crosswalk().stop_timeout()) {
              stop = false;
            }
          }
        }
      }

      if (stop) {//最后的结果是需要停车,则pedestrians.push_back(obstacle_id),供后续处理使用;
        pedestrians.push_back(obstacle_id);
        ADEBUG << "wait for: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      } else {
        ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      }
    }

    if (!pedestrians.empty()) {
      crosswalks_to_stop.emplace_back(crosswalk_overlap, pedestrians);
      ADEBUG << "crosswalk_id[" << crosswalk_id << "] STOP";
    }
  }

  double min_s = std::numeric_limits<double>::max();
  hdmap::PathOverlap* firsts_crosswalk_to_stop = nullptr;
  for (auto crosswalk_to_stop : crosswalks_to_stop) {
    // build stop decision
    const auto* crosswalk_overlap = crosswalk_to_stop.first;
    ADEBUG << "BuildStopDecision: crosswalk[" << crosswalk_overlap->object_id
           << "] start_s[" << crosswalk_overlap->start_s << "]";
    std::string virtual_obstacle_id =
        CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;
    util::BuildStopDecision(virtual_obstacle_id, crosswalk_overlap->start_s,
                            config_.crosswalk().stop_distance(),
                            StopReasonCode::STOP_REASON_CROSSWALK,
                            crosswalk_to_stop.second,
                            TrafficRuleConfig::RuleId_Name(config_.rule_id()),
                            frame, reference_line_info);

    //查找将要遇到的第一个人行横道
    if (crosswalk_to_stop.first->start_s < min_s) {
      firsts_crosswalk_to_stop =
          const_cast<PathOverlap*>(crosswalk_to_stop.first);
      min_s = crosswalk_to_stop.first->start_s;
    }
  }

  if (firsts_crosswalk_to_stop) {
    // update CrosswalkStatus
    std::string crosswalk = firsts_crosswalk_to_stop->object_id;
    mutable_crosswalk_status->set_crosswalk_id(crosswalk);
    mutable_crosswalk_status->clear_stop_time();
    for (const auto& timer : crosswalk_stop_timer[crosswalk]) {
      auto* stop_time = mutable_crosswalk_status->add_stop_time();
      stop_time->set_obstacle_id(timer.first);
      stop_time->set_stop_timestamp_sec(timer.second);
      ADEBUG << "UPDATE stop_time: id[" << crosswalk << "] obstacle_id["
             << timer.first << "] stop_timestamp[" << timer.second << "]";
    }

    // update CrosswalkStatus.finished_crosswalk
    mutable_crosswalk_status->clear_finished_crosswalk();
    for (auto crosswalk_overlap : crosswalk_overlaps_) {
      if (crosswalk_overlap->start_s < firsts_crosswalk_to_stop->start_s) {
        mutable_crosswalk_status->add_finished_crosswalk(
            crosswalk_overlap->object_id);
        ADEBUG << "UPDATE finished_crosswalk: " << crosswalk_overlap->object_id;
      }
    }
  }

  ADEBUG << "crosswalk_status: " << mutable_crosswalk_status->DebugString();
}


//找到所有人行横道
bool Crosswalk::FindCrosswalks(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  crosswalk_overlaps_.clear();
  const std::vector<hdmap::PathOverlap>& crosswalk_overlaps =
      reference_line_info->reference_line().map_path().crosswalk_overlaps();
  for (const hdmap::PathOverlap& crosswalk_overlap : crosswalk_overlaps) {
    crosswalk_overlaps_.push_back(&crosswalk_overlap);
  }
  return crosswalk_overlaps_.size() > 0;
}

bool Crosswalk::CheckStopForObstacle(
    ReferenceLineInfo* const reference_line_info,
    const CrosswalkInfoConstPtr crosswalk_ptr, const Obstacle& obstacle,
    const double stop_deceleration) {
  CHECK_NOTNULL(reference_line_info);

  std::string crosswalk_id = crosswalk_ptr->id().id();

  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  const std::string& obstacle_id = obstacle.Id();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);
  double adc_end_edge_s = reference_line_info->AdcSlBoundary().start_s();

  // check type
  //如果障碍物类型不是行人,自行车,未知移动物体或者位置类型
  if (obstacle_type != PerceptionObstacle::PEDESTRIAN &&
      obstacle_type != PerceptionObstacle::BICYCLE &&
      obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
      obstacle_type != PerceptionObstacle::UNKNOWN) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]. skip";
    return false;
  }

  // expand crosswalk polygon
  // note: crosswalk expanded area will include sideway area
  Vec2d point(perception_obstacle.position().x(),
              perception_obstacle.position().y());
  const Polygon2d crosswalk_exp_poly =
      crosswalk_ptr->polygon().ExpandByDistance(
          config_.crosswalk().expand_s_distance());//默认2m
  bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);//障碍物xy坐标是否在人行横道扩后的框内

  if (!in_expanded_crosswalk) {//不在则忽略
    ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
           << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
           << "]: not in crosswalk expanded area";
    return false;
  }

  const auto& reference_line = reference_line_info->reference_line();

  common::SLPoint obstacle_sl_point;
  reference_line.XYToSL(perception_obstacle.position(), &obstacle_sl_point);
  auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  const double obstacle_l_distance =//l的绝对值
      std::min(std::fabs(obstacle_sl_boundary.start_l()),
               std::fabs(obstacle_sl_boundary.end_l()));

  const bool is_on_lane =
      reference_line.IsOnLane(obstacle.PerceptionSLBoundary());
  const bool is_on_road =
      reference_line.IsOnRoad(obstacle.PerceptionSLBoundary());
  const bool is_path_cross = !obstacle.reference_line_st_boundary().IsEmpty();//如果障碍物的轨迹与参考线相交

  ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
         << "] crosswalk_id[" << crosswalk_id << "] obstacle_l["
         << obstacle_sl_point.l() << "] within_expanded_crosswalk_area["
         << in_expanded_crosswalk << "] obstacle_l_distance["
         << obstacle_l_distance << "] on_lane[" << is_on_lane << "] is_on_road["
         << is_on_road << "] is_path_cross[" << is_path_cross << "]";

  bool stop = false;
  if (obstacle_l_distance >= config_.crosswalk().stop_loose_l_distance()) {//默认8m
    // (1) when obstacle_l_distance is big enough(>= loose_l_distance),
    //     STOP only if paths crosses
    if (is_path_cross) {//如果障碍物的轨迹与参考线相交
      stop = true;
      ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle_id << "] type["
             << obstacle_type_name << "] crosswalk_id[" << crosswalk_id << "]";
    }
  } else if (obstacle_l_distance <=
             config_.crosswalk().stop_strict_l_distance()) {//默认6m
    if (is_on_road) {
      // (2) when l_distance <= strict_l_distance + on_road
      //     always STOP
      //当障碍物的横向距离小于比较严苛的横向停车距离,如果是前道路范围内的前向障碍物,则需要停车
      if (obstacle_sl_point.s() > adc_end_edge_s) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] s[" << obstacle_sl_point.s()
               << "] adc_end_edge_s[ " << adc_end_edge_s << "] crosswalk_id["
               << crosswalk_id << "] ON_ROAD";
      }
    } else {
      // (3) when l_distance <= strict_l_distance
      //     + NOT on_road(i.e. on crosswalk/median etc)
      //     STOP if paths cross
      if (is_path_cross) {
        //当障碍物的横向距离小于比较严苛的横向停车距离,如果是不在当前道路范围内但轨迹与参考线相交的障碍物,则需要停车
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "] PATH_CRSOSS";
      } else {
        //当障碍物的横向距离小于比较严苛的横向停车距离,如果是不在当前道路范围内但正在本车靠近的障碍物,则需要停车
        // (4) when l_distance <= strict_l_distance
        //     + NOT on_road(i.e. on crosswalk/median etc)
        //     STOP if he pedestrian is moving toward the ego vehicle
        const auto obstacle_v = Vec2d(perception_obstacle.velocity().x(),
                                      perception_obstacle.velocity().y());
        const auto adc_path_point =
            Vec2d(injector_->ego_info()->start_point().path_point().x(),
                  injector_->ego_info()->start_point().path_point().y());
        const auto ovstacle_position =
            Vec2d(perception_obstacle.position().x(),
                  perception_obstacle.position().y());
        auto obs_to_adc = adc_path_point - ovstacle_position;
        const double kEpsilon = 1e-6;
        if (obstacle_v.InnerProd(obs_to_adc) > kEpsilon) {
          stop = true;
          ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
                 << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
                 << "] MOVING_TOWARD_ADC";
        }
      }
    }
  } else {
    // (4) when l_distance is between loose_l and strict_l
    //     use history decision of this crosswalk to smooth unsteadiness

    // TODO(all): replace this temp implementation
    //当障碍物的横向距离大于比较严苛的横向停车距离同时小于一定的阈值(宽松的停车距离)时,如果障碍物与参考线轨迹相交,则停车;
    if (is_path_cross) {
      stop = true;
    }
    ADEBUG << "need_stop(between l1 & l2): obstacle_id[" << obstacle_id
           << "] type[" << obstacle_type_name << "] obstacle_l_distance["
           << obstacle_l_distance << "] crosswalk_id[" << crosswalk_id
           << "] USE_PREVIOUS_DECISION";
  }

  // check stop_deceleration
  if (stop) {
    //如果(iii)处理的结果需要停车,并且停车减速度大于配置文件给出的最大减速度,同时障碍物的横向距离大于比较严苛的横向停车距离,则不再需要停车
    if (stop_deceleration >= config_.crosswalk().max_stop_deceleration()) {
      if (obstacle_l_distance > config_.crosswalk().stop_strict_l_distance()) {
        // SKIP when stop_deceleration is too big but safe to ignore
        stop = false;
      }
      AWARN << "crosswalk_id[" << crosswalk_id << "] stop_deceleration["
            << stop_deceleration << "]";
    }
  }

  return stop;
}

}  // namespace planning
}  // namespace apollo
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

#include "modules/planning/traffic_rules/backside_vehicle.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
//后向来车的处理决策
BacksideVehicle::BacksideVehicle(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

void BacksideVehicle::MakeLaneKeepingObstacleDecision(
    const SLBoundary& adc_sl_boundary, PathDecision* path_decision) {
  ObjectDecisionType ignore;
  ignore.mutable_ignore();
  const double adc_length_s =
      adc_sl_boundary.end_s() - adc_sl_boundary.start_s();
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    //对于前方障碍物及警戒级别的障碍物不作处理
    if (obstacle->PerceptionSLBoundary().end_s() >= adc_sl_boundary.end_s() ||
        obstacle->IsCautionLevelObstacle()) {
      // don't ignore such vehicles.
      continue;
    }
    //如果参考线上没有障碍物运动轨迹,则忽略
    if (obstacle->reference_line_st_boundary().IsEmpty()) {
      path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region",
                                             obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/no-st-region",
                                        obstacle->Id(), ignore);
      continue;
    }
    // Ignore the car comes from back of ADC
    //如果障碍物轨迹距离无人车的最近距离小于一个车长,则忽略
    if (obstacle->reference_line_st_boundary().min_s() < -adc_length_s) {
      path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc",
                                             obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc",
                                        obstacle->Id(), ignore);
      continue;
    }

    const double lane_boundary =
        config_.backside_vehicle().backside_lane_width();//默认4m
    if (obstacle->PerceptionSLBoundary().start_s() < adc_sl_boundary.end_s()) {
      if (obstacle->PerceptionSLBoundary().start_l() > lane_boundary ||
          obstacle->PerceptionSLBoundary().end_l() < -lane_boundary) {
        continue;
      }
      //如果距离本车横向距离小于一定阈值的后向车辆,说明车辆不会超车.选择忽略
      path_decision->AddLongitudinalDecision("backside_vehicle/sl < adc.end_s",
                                             obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/sl < adc.end_s",
                                        obstacle->Id(), ignore);
      continue;
    }
  }
}

Status BacksideVehicle::ApplyRule(
    Frame* const, ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  // The lane keeping reference line.
  if (reference_line_info->Lanes().IsOnSegment()) {
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo

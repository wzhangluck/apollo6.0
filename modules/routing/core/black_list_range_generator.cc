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

#include "modules/routing/core/black_list_range_generator.h"

namespace apollo {
namespace routing {

constexpr double S_GAP_FOR_BLACK = 0.01;

namespace {


//把s往前移动一个距离
double MoveSForward(double s, double upper_bound) {
  if (s > upper_bound) {
    AERROR << "Illegal s: " << s << ", upper bound: " << upper_bound;
    return s;
  }
  if (s + S_GAP_FOR_BLACK < upper_bound) {
    return (s + S_GAP_FOR_BLACK);
  } else {
    return ((s + upper_bound) / 2.0);
  }
}

//把s往回移动一个距离S_GAP_FOR_BLACK，下限是lower_bound
//此处S_GAP_FOR_BLACK=0.01，lower_bound为0，意味着s位置基本未变
double MoveSBackward(double s, double lower_bound) {
  if (s < lower_bound) {
    AERROR << "Illegal s: " << s << ", lower bound: " << lower_bound;
    return s;
  }
  if (s - S_GAP_FOR_BLACK > lower_bound) {
    return (s - S_GAP_FOR_BLACK);
  } else {
    return ((s + lower_bound) / 2.0);
  }
}

//遍历node的每一条左右侧边（自身node是边的终点)，根据边获取边的终点node，保存，递归检索出发节点的边和它的出发节点
void GetOutParallelLane(const TopoNode* node,
                        std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->OutToLeftOrRightEdge()) {
    const auto* to_node = edge->ToNode();
    if (node_set->count(to_node) == 0) {
      node_set->emplace(to_node);
      GetOutParallelLane(to_node, node_set);
    }
  }
}

//遍历node的每一条边（自身node是边的终点)，根据边获取边的出发node，保存，递归检索出发节点的边和它的出发节点
void GetInParallelLane(const TopoNode* node,
                       std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->InFromLeftOrRightEdge()) {
    const auto* from_node = edge->FromNode();
    if (node_set->count(from_node) == 0) {
      node_set->emplace(from_node);
      GetInParallelLane(from_node, node_set);
    }
  }
}

// for new navigator
//查找道路id所包含的node节点
void AddBlackMapFromRoad(const RoutingRequest& request, const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  for (const auto& road_id : request.blacklisted_road()) {
    std::unordered_set<const TopoNode*> road_nodes_set;
    graph->GetNodesByRoadId(road_id, &road_nodes_set);
    for (const auto& node : road_nodes_set) {
      range_manager->Add(node, 0.0, node->Length());
    }
  }
}

// for new navigator
//根据RoutingRequest的黑名单车道，得到对应的node，添加到TopoRangeManager中，包括车道节点及对应的起始-终止s范围
void AddBlackMapFromLane(const RoutingRequest& request, const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  for (const auto& lane : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(lane.id());
    if (node) {
      range_manager->Add(node, lane.start_s(), lane.end_s());
    }
  }
}

//把该车道的子节点车道加入到par_node_set
void AddBlackMapFromOutParallel(const TopoNode* node, double cut_ratio,
                                TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetOutParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  //遍历子节点，加入
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}
//把该车道的父节点的车道加入到par_node_set
void AddBlackMapFromInParallel(const TopoNode* node, double cut_ratio,
                               TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetInParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

}  // namespace

//把黑名单的道路和车道id添加到range_manager
void BlackListRangeGenerator::GenerateBlackMapFromRequest(
    const RoutingRequest& request, const TopoGraph* graph,
    TopoRangeManager* const range_manager) const {
  AddBlackMapFromLane(request, graph, range_manager);
  AddBlackMapFromRoad(request, graph, range_manager);
  range_manager->SortAndMerge();
}

void BlackListRangeGenerator::AddBlackMapFromTerminal(
    const TopoNode* src_node, const TopoNode* dest_node, double start_s,
    double end_s, TopoRangeManager* const range_manager) const {
  double start_length = src_node->Length();
  double end_length = dest_node->Length();

  static constexpr double kEpsilon = 1e-2;
  //注意：每条车道的start_s都是按照初始点s为0计算的
  //如果s＞车道长度，并且未超出太多，取start_s_adjusted为start_length，否则仍为start_s
  const double start_s_adjusted =
      (start_s > start_length && start_s - start_length <= kEpsilon) ?
          start_length : start_s;
  const double end_s_adjusted =
      (end_s > end_length && end_s - end_length <= kEpsilon) ?
          end_length : end_s;

  if (start_s_adjusted < 0.0 || start_s_adjusted > start_length) {
    AERROR << "Illegal start_s: " << start_s << ", length: " << start_length;
    return;
  }
  if (end_s_adjusted < 0.0 || end_s_adjusted > end_length) {
    AERROR << "Illegal end_s: " << end_s << ", length: " << end_length;
    return;
  }

  double start_cut_s = MoveSBackward(start_s_adjusted, 0.0);
  //该节点增加一个range，起始终止同一个点，因此range就是一个点
  range_manager->Add(src_node, start_cut_s, start_cut_s);
  //range_manager添加左右平行车道节点及各自的range（start_cut_s / start_length）
  AddBlackMapFromOutParallel(src_node, start_cut_s / start_length,
                             range_manager);

  double end_cut_s = MoveSForward(end_s_adjusted, end_length);
  range_manager->Add(dest_node, end_cut_s, end_cut_s);
  AddBlackMapFromInParallel(dest_node, end_cut_s / end_length, range_manager);
  range_manager->SortAndMerge();
}

}  // namespace routing
}  // namespace apollo

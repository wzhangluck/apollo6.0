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

#include "modules/routing/graph/topo_range_manager.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace routing {
namespace {

//把所有节点的s进行合并，得到黑名单的s范围
void merge_block_range(const TopoNode* topo_node,
                       const std::vector<NodeSRange>& origin_range,
                       std::vector<NodeSRange>* block_range) {
  std::vector<NodeSRange> sorted_origin_range(origin_range);
  //按照每一个的start的s进行排序
  std::sort(sorted_origin_range.begin(), sorted_origin_range.end());
  size_t cur_index = 0;
  auto total_size = sorted_origin_range.size();//节点个数
  while (cur_index < total_size) {
    NodeSRange range(sorted_origin_range[cur_index]);
    ++cur_index;
    while (cur_index < total_size &&
           range.MergeRangeOverlap(sorted_origin_range[cur_index])) {//合并两者s
      ++cur_index;
    }
    if (range.EndS() < topo_node->StartS() ||
        range.StartS() > topo_node->EndS()) {
      continue;
    }
    range.SetStartS(std::max(topo_node->StartS(), range.StartS()));
    range.SetEndS(std::min(topo_node->EndS(), range.EndS()));
    block_range->push_back(std::move(range));
  }
}

}  // namespace

const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>&
TopoRangeManager::RangeMap() const {
  return range_map_;
}
const std::vector<NodeSRange>* TopoRangeManager::Find(
    const TopoNode* node) const {
  auto iter = range_map_.find(node);
  if (iter == range_map_.end()) {
    return nullptr;
  } else {
    return &(iter->second);
  }
}

void TopoRangeManager::PrintDebugInfo() const {
  for (const auto& map : range_map_) {
    for (const auto& range : map.second) {
      AINFO << "black lane id: " << map.first->LaneId()
            << ", start s: " << range.StartS() << ", end s: " << range.EndS();
    }
  }
}

void TopoRangeManager::Clear() { range_map_.clear(); }


//把节点及相应的起始-终止s加入到range_map_
void TopoRangeManager::Add(const TopoNode* node, double start_s, double end_s) {
  NodeSRange range(start_s, end_s);
  range_map_[node].push_back(range);
}

//遍历车道std::unordered_map<const TopoNode*, std::vector<NodeSRange>>
//输入的RoutingRequest中可能会有多个车道node，对应有不同的range_map_，这里对其进行合并
void TopoRangeManager::SortAndMerge() {
  for (auto& iter : range_map_) {
    std::vector<NodeSRange> merged_range_vec;
    merge_block_range(iter.first, iter.second, &merged_range_vec);
    iter.second.assign(merged_range_vec.begin(), merged_range_vec.end());
  }
}

}  // namespace routing
}  // namespace apollo

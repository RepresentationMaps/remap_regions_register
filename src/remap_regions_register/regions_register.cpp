// Copyright 2025 PAL Robotics, S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <algorithm>

#include "remap_regions_register/regions_register.hpp"

namespace remap
{
namespace regions_register
{
RegionsRegister::RegionsRegister(const bool & threaded)
: threaded_(threaded),
  id_(-1) {}

RegionsRegister::~RegionsRegister() {}

int RegionsRegister::addArea(const std::vector<std::string> & regs)
{
  // adds a new area assigned to the set of regions regs;
  // The logic is that it always tries to assign the lowest possible
  // id (>= 0).
  std::lock_guard<std::recursive_mutex> guard(register_mutex_);


  id_++;
  int id;
  if (lookup_areas_.size() == 0) {
    // Since no areas are stored, id 0 is available
    id = 0;
  } else if (lookup_areas_.begin()->first != 0) {
    // The area with ID 0 does not exist;
    // it is possible to use ID 0
    id = 0;
  } else {
    auto lookup_it = lookup_areas_.begin();
    while ((lookup_it != lookup_areas_.end()) && (std::next(lookup_it) != lookup_areas_.end())) {
      if ((lookup_it->first + 1) != (std::next(lookup_it))->first) {
        id = lookup_it->first + 1;
        break;
      }
      lookup_it++;
    }
    if (std::next(lookup_it) == lookup_areas_.end()) {
      id = lookup_it->first + 1;
    }
  }
  areas_[regs] = id;
  lookup_areas_[id] = regs;
  return id;
}

std::map<int, int> RegionsRegister::removeRegion(const std::string & reg)
{
  // remove an entire region from the register
  std::vector<std::vector<std::string>> areas_to_remove;
  std::map<std::vector<std::string>, int> areas_to_add;
  std::map<int, int> ids_to_update;
  std::lock_guard<std::recursive_mutex> guard(register_mutex_);
  bool standalone_reg_removed = false;
  for (const auto & area : areas_) {
    auto regs = area.first;
    auto reg_elem = std::find(regs.begin(), regs.end(), reg);
    if (reg_elem == regs.end()) {
      continue;
    }
    // At this point, we know that the region we are removing
    // is contained inside of this area;
    // Therefore, we add this area to those to remove
    areas_to_remove.push_back(regs);
    regs.erase(reg_elem);
    if (regs.size() == 0) {
      // We just found the register entrance
      // cotaining only the region we are deleting.
      // We can remove the BBox associated with this ID.
      if (!standalone_reg_removed) {
        auto id_to_remove = areas_.find({reg});
        if (id_to_remove != areas_.end()) {
          lookup_areas_.erase(id_to_remove->second);
          standalone_reg_removed = true;
        }
      }
      continue;
    }
    int old_id = area.second;
    int new_id = findRegions(regs);
    if (new_id == -1) {
      // This is the case where we are removing
      // an entity example_123 from the register
      // and we are iterating over the area
      // {example_123, example_456}. If the area
      // {example_456} does not exist, we assign
      // the area {example_456} to the current
      // {example_123, example_456} ID. This
      // operation does not affect the BBoxes size
      areas_to_add.insert(std::pair(regs, old_id));
    } else {
      // This is the case where we are removing
      // an entity example_123 from the register
      // and we are iterating over the area
      // {example_123, example_456}. If the area
      // {example_456} exists, we have to update the
      // value stored in those voxels containing
      // the ID assigned to {example_123, example_456}
      // with the ID assigned to {example_456}.
      // We have to enlarge the {example_456} BBox
      // and remove the {example_123, example_456} BBox
      ids_to_update.insert(std::pair<int, int>(old_id, new_id));
      // we can also remove the old_id from the id_reg
      lookup_areas_.erase(old_id);
    }
  }
  // We proceed removing the elements from the areas, according
  // to the previous iteration
  for (const auto & area : areas_to_remove) {
    areas_.erase(area);
  }
  // We complete the updated by adding those new areas
  // created by the removal of this RegionOfSpace
  for (const auto & area : areas_to_add) {
    areas_.insert(area);
  }
  return ids_to_update;
}

int RegionsRegister::findRegions(const std::vector<std::string> & regs) const
{
  auto elem = areas_.find(regs);
  if (elem == areas_.end()) {
    return -1;
  } else {
    return elem->second;
  }
}

std::vector<std::string> RegionsRegister::findRegionsById(const int & id) const
{
  std::vector<std::string> regs;
  for (auto elem : areas_) {
    if (elem.second == id) {
      regs = elem.first;
      break;
    }
  }
  return regs;
}

std::map<std::vector<std::string>, int> RegionsRegister::getAreas() const
{
  return areas_;
}

std::vector<int> RegionsRegister::getEntityIds(const std::string & entity) const
{
  std::vector<int> ids;
  for (const auto & area : areas_) {
    if (std::find(area.first.begin(), area.first.end(), entity) != area.first.end()) {
      ids.push_back(area.second);
    }
  }
  return ids;
}

void RegionsRegister::clear()
{
  areas_.clear();
  id_ = -1;
}

int RegionsRegister::getRegionsNumber() const
{
  return areas_.size();
}

void RegionsRegister::print() const
{
  for (auto elem : areas_) {
    std::cout << "Area " << elem.second << ": ";
    for (auto reg : elem.first) {
      std::cout << reg << " ";
    }
    std::cout << std::endl;
  }
}

int RegionsRegister::getId() const
{
  return id_;
}

std::vector<std::string> RegionsRegister::getInstances() const
{
  std::vector<std::string> instances;
  for (const auto & area : areas_) {
    for (const auto & instance : area.first) {
      if (std::find(instances.begin(), instances.end(), instance) == instances.end()) {
        instances.push_back(instance);
      }
    }
  }
  return instances;
}

std::vector<std::vector<std::string>> RegionsRegister::getEntries() const
{
  std::vector<std::vector<std::string>> entries;
  for (const auto & area : areas_) {
    entries.push_back(area.first);
  }
  return entries;
}

void RegionsRegister::addEntityType(const std::string & entity, const std::string & type)
{
  entities_types_[entity] = type;
}

std::string RegionsRegister::getEntityType(const std::string & entity)
{
  if (entities_types_.find(entity) != entities_types_.end()) {
    return entities_types_[entity];
  } else {
    return "";
  }
}

std::unordered_set<std::string> RegionsRegister::getCoexistentEntities(const std::string & entity)
{
  std::unordered_set<std::string> coexisting;

  for (const auto & region : areas_) {
    if (std::find(region.first.begin(), region.first.end(), entity) != region.first.end()) {
      for (const auto & coexisting_entity : region.first) {
        coexisting.insert(coexisting_entity);
      }
    }
  }

  coexisting.erase(entity);
  return coexisting;
}

}  // namespace regions_register
}  // namespace remap

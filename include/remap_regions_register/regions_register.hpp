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

#ifndef REMAP_REGIONS_REGISTER__REGIONS_REGISTER_HPP_
#define REMAP_REGIONS_REGISTER__REGIONS_REGISTER_HPP_

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <mutex>

namespace remap
{
namespace regions_register
{
class RegionsRegister
{
protected:
  bool threaded_;
  std::recursive_mutex register_mutex_;
  std::map<std::vector<std::string>, int> areas_;
  std::map<int, std::vector<std::string>> lookup_areas_;
  std::map<std::string, std::string> entities_types_;
  int id_;

public:
  explicit RegionsRegister(const bool & threaded);
  ~RegionsRegister();

  int addArea(const std::vector<std::string> & regs);
  std::map<int, int> removeRegion(const std::string & reg);
  int findRegions(const std::vector<std::string> & regs) const;
  std::vector<std::string> findRegionsById(const int & id) const;
  void clear();

  // Debugging functions
  int getRegionsNumber() const;
  void print() const;
  int getId() const;

  std::vector<std::string> getInstances() const;
  std::vector<std::vector<std::string>> getEntries() const;

  void addEntityType(const std::string & entity, const std::string & type);
  std::string getEntityType(const std::string & entity);
};
}  // namespace regions_register
}  // namespace remap
#endif  // REMAP_REGIONS_REGISTER__REGIONS_REGISTER_HPP_

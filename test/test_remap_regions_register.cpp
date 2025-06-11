// Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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

#include <map>
#include <memory>

#include "gtest/gtest.h"
#include "remap_regions_register/regions_register.hpp"

class RRRTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {}

  static void TearDownTestSuite()
  {}

  void SetUp() override
  {
    regions_register_ = std::make_shared<remap::regions_register::RegionsRegister>(false);
  }

  void TearDown() override
  {
    regions_register_->clear();
    regions_register_.reset();
  }

  std::shared_ptr<remap::regions_register::RegionsRegister> regions_register_;
};

TEST_F(RRRTest, AddElements)
{
  regions_register_->addArea({"abc", "def"});
  regions_register_->addArea({"abc"});
  regions_register_->addArea({"def", "ghi"});
  regions_register_->addArea({"jkl", "mno", "pqr"});

  ASSERT_EQ(regions_register_->getRegionsNumber(), 4);
  ASSERT_EQ(regions_register_->getId(), 3);
}

TEST_F(RRRTest, RemoveElements)
{
  regions_register_->addArea({"abc"});
  regions_register_->addArea({"abc", "def"});
  regions_register_->addArea({"def", "ghi"});
  regions_register_->addArea({"jkl", "mno", "pqr"});

  auto ids_to_update = regions_register_->removeRegion({"def"});

  ASSERT_EQ(regions_register_->getRegionsNumber(), 3);
  ASSERT_EQ(ids_to_update.size(), 1);
  ASSERT_EQ(ids_to_update.begin()->first, 1);
  ASSERT_EQ(ids_to_update.begin()->second, 0);
}

TEST_F(RRRTest, FindElements)
{
  regions_register_->addArea({"abc", "def"});
  regions_register_->addArea({"abc"});
  regions_register_->addArea({"def", "ghi"});
  regions_register_->addArea({"jkl", "mno", "pqr"});

  ASSERT_EQ(regions_register_->findRegions({"abc", "def"}), 0);

  auto ids_to_update = regions_register_->removeRegion({"def"});

  ASSERT_EQ(regions_register_->findRegions({"abc", "def"}), -1);
  ASSERT_EQ(regions_register_->findRegions({"abc"}), 1);

  regions_register_->addArea({"mno"});

  ASSERT_EQ(regions_register_->findRegions({"mno"}), 0);
}

TEST_F(RRRTest, FindRegionsByID)
{
  regions_register_->addArea({"abc"});
  regions_register_->addArea({"abc", "def"});
  regions_register_->addArea({"def", "ghi"});
  regions_register_->addArea({"jkl", "mno", "pqr"});

  ASSERT_EQ(regions_register_->findRegionsById(0), std::vector<std::string>({"abc"}));
  ASSERT_EQ(regions_register_->findRegionsById(1), std::vector<std::string>({"abc", "def"}));
  ASSERT_EQ(regions_register_->findRegionsById(2), std::vector<std::string>({"def", "ghi"}));
  ASSERT_EQ(regions_register_->findRegionsById(3), std::vector<std::string>({"jkl", "mno", "pqr"}));

  auto ids_to_update = regions_register_->removeRegion({"def"});

  ASSERT_EQ(regions_register_->findRegionsById(0), std::vector<std::string>({"abc"}));
  ASSERT_EQ(regions_register_->findRegionsById(1), std::vector<std::string>({}));
  ASSERT_EQ(regions_register_->findRegionsById(2), std::vector<std::string>({"ghi"}));
  ASSERT_EQ(regions_register_->findRegionsById(3), std::vector<std::string>({"jkl", "mno", "pqr"}));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

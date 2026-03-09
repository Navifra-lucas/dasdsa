#include "core_agent/core_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <gtest/gtest.h>

namespace NaviFra {
class CoreAgentMemoryRepository : public ::testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

TEST_F(CoreAgentMemoryRepository, any)
{
    InMemoryRepository& repository = InMemoryRepository::instance();

    auto status = std::make_shared<RobotStatus>();
    repository.add(status->KEY, status);
    status->setID("1111");

    EXPECT_EQ(true, repository.exists<RobotStatus>(status->KEY));

    auto retrievedStatus = repository.get<RobotStatus>(status->KEY);
    EXPECT_EQ(true, retrievedStatus.get() != nullptr);

    retrievedStatus->setID("222");
    repository.update(retrievedStatus->KEY, retrievedStatus);

    auto retrievedStatus2 = repository.get<RobotStatus>(status->KEY);
    EXPECT_EQ(true, retrievedStatus2.get() != nullptr);
    EXPECT_EQ("222", retrievedStatus2->getID());

    retrievedStatus->setID("333");

    auto retrievedStatus3 = repository.get<RobotStatus>(status->KEY);
    EXPECT_EQ("333", retrievedStatus3->getID());

    repository.remove<RobotStatus>(retrievedStatus->KEY);
    EXPECT_EQ(false, repository.exists<RobotStatus>(retrievedStatus->KEY));
}
}  // namespace NaviFra
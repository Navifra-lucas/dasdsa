#include "nc_brain_agent/nc_brain_agent.h"
#include "nc_brain_agent/nc_robot_agent.h"
#include "nc_brain_map_test.h"
#include "nc_brain_test_server.h"

#include <Poco/File.h>
#include <Poco/JSON/Parser.h>
#include <Poco/PipeStream.h>
#include <Poco/Process.h>
#include <Poco/Redis/Command.h>
#include <gtest/gtest.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/net/nc_rest_api_utils.h>
#include <signal.h>

namespace NaviFra {
class BrainRobotAgentFixture : public ::testing::Test {
protected:
    void SetUp() override
    {
        try {
            srv.reset(new HTTPServer(new BrainTransportHandlerFactory, 5000));
            srv->start();

            Poco::Process::Args args;
            args.push_back(std::string(TEST_FILE_PATH) + "redis.conf");

            Poco::Pipe outPipe;
            Poco::ProcessHandle ph = Poco::Process::launch("redis-server", args);

            Config::instance().setString("backend_host", "127.0.0.1");
            Config::instance().setInt("backend_port", 5000);

            pid_ = ph.id();
        }
        catch (std::exception ex) {
            NLOG(error) << ex.what();
        }
    }

    void TearDown() override
    {
        srv->stop();
        srv.reset();
        Poco::Process::requestTermination(pid_);

        NcBrainMap::Ptr map(new NcBrainMap());
        Poco::File(map->getDir()).remove(true);
    }

private:
    std::shared_ptr<HTTPServer> srv;
    Poco::Process::PID pid_;
};

TEST_F(BrainRobotAgentFixture, initialize)
{
    /// Redis Server가 뜨기까지 대기 서버 상황에 따라 다를 듯?
    int argc = 0;
    char** argv;
    ros::init(argc, argv, "nc_brain_agent");
    Poco::Thread::sleep(2000);
    Poco::Redis::Client redis;
    redis.connect("127.0.0.1", 6379, 10000);
    Poco::Redis::Array auth;
    auth.add("AUTH").add("navifra1@3$");
    std::string reply = redis.execute<std::string>(auth);
    Poco::Redis::Command command = Poco::Redis::Command::set("robot_acs_reg_code", "62d3b8b5-2ba3-4725-ace4-91d6fb54f81e");
    std::string result = redis.execute<std::string>(command);

    EXPECT_EQ(result.compare("OK") == 0, true);
    EXPECT_EQ(NcRobotAgent::get().initialize(), true);

    Poco::Thread::sleep(2000);
    NcRobotAgent::get().finalize();
}

}  // namespace NaviFra
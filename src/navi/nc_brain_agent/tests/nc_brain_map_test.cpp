#include "nc_brain_map_test.h"

#include "gtest/gtest.h"
#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Dynamic/Struct.h>
#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/Format.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/StreamCopier.h>
#include <nc_brain_agent/data/nc_brain_map.h>

namespace NaviFra {

class BrainMapFixture : public ::testing::Test {
protected:
    void SetUp() override
    {
        NcBrainMap::Ptr map(new NcBrainMap());
        Poco::File(map->getDir()).createDirectories();
        std::string filepath = std::string(TEST_FILE_PATH) + "test_map.json";
        Poco::File sourceFile(filepath);
        sourceFile.copyTo(map->getFilePath());
    }

    void TearDown() override
    {
        NcBrainMap::Ptr map(new NcBrainMap());
        Poco::File(map->getDir()).remove(true);
    }
};

TEST_F(BrainMapFixture, Mapinit)
{
    NcBrainMap::Ptr map(new NcBrainMap());
    EXPECT_TRUE(map->initialize());
    EXPECT_EQ(map->getLocalRevision(), "");
}

TEST_F(BrainMapFixture, getsetMainRevision)
{
    NcBrainMap::Ptr map(new NcBrainMap());

    map->setMainRevision("testrevision");
    EXPECT_EQ(map->getMainRevision(), "testrevision");
}

TEST_F(BrainMapFixture, localMapUpdate)
{
    NcBrainMap::Ptr map(new NcBrainMap());

    std::ostringstream ostr;
    Poco::FileInputStream fis(map->getFilePath());
    Poco::StreamCopier::copyStream(fis, ostr);
    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(ostr.str());
    Poco::JSON::Object::Ptr map_data = result.extract<Poco::JSON::Object::Ptr>();

    map_data->set("revision", "testrevision");
    map->localMapUpdate(map_data);
    fis.close();

    map->initialize();

    EXPECT_EQ(map->getLocalRevision(), "testrevision");
}

TEST_F(BrainMapFixture, getsetLocalRevision)
{
    NcBrainMap::Ptr map(new NcBrainMap());
    map->initialize();

    map->setLocalRevision("testrevision");
    EXPECT_EQ(map->getLocalRevision(), "testrevision");
    map->setLocalRevision("testrevision1");
    EXPECT_EQ(map->getLocalRevision(), "testrevision1");
    map->setLocalRevision("testrevision2");
    EXPECT_EQ(map->getLocalRevision(), "testrevision2");
    map->setLocalRevision("testrevision3");
    EXPECT_EQ(map->getLocalRevision(), "testrevision3");
    map->setLocalRevision("testrevision4");
    EXPECT_EQ(map->getLocalRevision(), "testrevision4");
}

TEST_F(BrainMapFixture, getSlamMap)
{
    NcBrainMap::Ptr map(new NcBrainMap());
    map->initialize();

    // EXPECT_EQ(map->getSlamMap(), nullptr);
}

TEST_F(BrainMapFixture, isSlamFileModified)
{
    NcBrainMap::Ptr map(new NcBrainMap());
    map->initialize();

    EXPECT_EQ(map->isSlamFileModified(), false);

    std::string slam_map_path = Poco::format("%s/navifra_solution/navicore/configs/map/temp.json", std::string(std::getenv("HOME")));
    Poco::File(slam_map_path).createDirectories();
    std::string filepath = Poco::format("%stest_map.json", std::string(std::string(std::string(TEST_FILE_PATH))));
    Poco::File sourceFile(filepath);
    sourceFile.copyTo(slam_map_path);

    EXPECT_EQ(map->isSlamFileModified(), true);

    Poco::File(slam_map_path).remove(true);
}

TEST_F(BrainMapFixture, getKeyNodesSize)
{
    NcBrainMap::Ptr map(new NcBrainMap());
    EXPECT_EQ(map->getKeyNodesSize(), 0);
}

TEST_F(BrainMapFixture, teachingUpdate)
{
    // GTEST_NONFATAL_FAILURE_("teachingUpdate 테스트 코드 작성 필요");
    // ASSERT_TRUE(true);
}

TEST_F(BrainMapFixture, teachingJsonInit)
{
    //  GTEST_NONFATAL_FAILURE_("teachingJsonInit 테스트 코드 작성 필요");
    //  ASSERT_TRUE(true);
}

TEST_F(BrainMapFixture, getTeachedNodesSize)
{
    NcBrainMap::Ptr map(new NcBrainMap());
    EXPECT_EQ(map->getTeachedNodesSize(), 0);

    std::string filepath = Poco::format("%stest_map.json", std::string(std::string(std::string(TEST_FILE_PATH))));
    Poco::File sourceFile(filepath);
    sourceFile.copyTo(Poco::format("%steaching.json", map->getDir()));
    EXPECT_EQ(map->getTeachedNodesSize(), 32);
}

}  // namespace NaviFra
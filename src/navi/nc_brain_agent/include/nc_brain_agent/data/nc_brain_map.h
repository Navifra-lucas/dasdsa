#ifndef NC_BRAIN_MAP_H
#define NC_BRAIN_MAP_H

#include <Poco/JSON/Object.h>

using Poco::JSON::Object;

namespace NaviFra {

    
struct MapPaths {
    std::string dir      = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    std::string map_json = dir + "map.json";
    std::string cur_txt  = dir + "current_area.txt";
    std::string areas_path = dir + "areas.json";
};

class NcBrainMap {
public:
    NcBrainMap();
    ~NcBrainMap();
    using Ptr = std::shared_ptr<NcBrainMap>;

    const static std::string KEY;

private:
    virtual std::string implName() { return "NcBrainMap"; }

public:
    bool initialize();
    std::string getDir() { return dirpath_; }
    std::string getFilePath() { return filepath_; }
    std::string getLocalRevision();
    void setLocalRevision(std::string revision);
    void localMapUpdate(Object::Ptr map);
    void localMapUpdateDir(Object::Ptr map, std::string s_filepath);
    void updateAreaPortal();
    void ChageCurrentArea(const std::string& area_id);
    std::string getMainRevision() { return main_revision_; }
    void setMainRevision(std::string revision);
    void teachingJsonInit(std::string revision);
    bool loadLocalMap();
    bool isSlamFileModified();

    size_t getKeyNodesSize();
    Object::Ptr getSlamMap();

    size_t getTeachedNodesSize();
    Object::Ptr getTeachedData();
    std::string teachingUpdate(Object::Ptr teachNodes);

    bool getNodePositionByNodeid(std::string node_id, float& node_x, float& node_y, float& node_deg);
    bool getNodePositionNearbyRobot(std::string& node_id, float& node_x, float& node_y, float& node_deg);
    bool getLinkDriveData(std::string start_node_id, std::string goal_node_id, float& linear_speed, int& drive_type);

private:
    std::string main_revision_;
    std::string local_revision_;
    std::string dirpath_;
    std::string filepath_;
    std::string teaching_;

    Poco::Timestamp slam_map_lastmodified_;

    Object::Ptr map_data_;
};
}  // namespace NaviFra

#endif  // NC_BRAIN_MAP_H
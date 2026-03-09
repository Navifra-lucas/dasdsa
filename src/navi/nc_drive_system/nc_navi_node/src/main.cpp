#include "nc_navi_node/NaviCANNode.hpp"

using namespace NaviFra::NaviCAN::Node;

int main(int argc, char** argv)
{

    try 
    { 
        ros::init(argc, argv, "nc_navi_node");
        
        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");
        NaviCANNode node(nh, nhp);
       if (!node.initialize()) {
            NLOG(info) << "Failed to initialize NaviCANNode.";
            return -1;
        }
        
        ros::spin();

        node.finalize();
        return 0;
        
    } catch (const std::exception& e) {
        NLOG(severity_level::error) << "Fatal error: " << e.what();
        return -1;
    } catch (...) {
        NLOG(severity_level::error) << "Unknown fatal error occurred";
        return -1;
    }

    return 0;
}
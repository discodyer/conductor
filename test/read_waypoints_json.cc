#include <ros/ros.h>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "conductor/way_point.h" // 假设 WaypointManager 类的头文件在这里包含

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_reader_node");
    ros::NodeHandle nh;

    std::string jsonFilePath = "./src/conductor/example/json/waypoints.json"; // 修改为你的 JSON 文件路径

    // 创建 WaypointManager 实例，并从 JSON 文件读取航点数据
    WaypointManager waypointManager(jsonFilePath);

    // 输出航点数据
    while (ros::ok())
    {
        waypointManager.printCurrentWaypoint();
        ros::Duration(1.0).sleep(); // 每秒输出一次航点数据
        ros::spinOnce();
        way_point::Waypoint w;
        if (!waypointManager.getNextWaypoint(w))
        {
            break;
        }
    }
    ros::shutdown();
    return 0;
}
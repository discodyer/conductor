#include <ros/ros.h>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "conductor/waypoint.h" // 假设 WaypointManager 类的头文件在这里包含

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_reader_node");
    ros::NodeHandle nh;

    std::string transformJsonFilePath = "./src/conductor/example/json/transforms.json"; // 修改为你的 JSON 文件路径

    FrameManager transformManager(transformJsonFilePath);

    std::string waypointJsonFilePath = "./src/conductor/example/json/rectangle_waypoint.json"; // 修改为你的 JSON 文件路径

    // 创建 WaypointManager 实例，并从 JSON 文件读取航点数据
    WaypointManager waypointManager(waypointJsonFilePath);

    ros::Rate rate(10);

    while(ros::ok())
    {
        transformManager.printFrameInfoALL();
        break;
    }

    // 输出航点数据
    while (ros::ok())
    {
        waypointManager.printCurrentWaypointLoop();
        if(waypointManager.isWaypointDelaySatisfied())
        {
            if(!waypointManager.goToNextWaypoint())
            {
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}
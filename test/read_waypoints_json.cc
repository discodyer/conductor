#include <ros/ros.h>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "conductor/waypoint.h" // 假设 WaypointManager 类的头文件在这里包含
#include "conductor/ansi_color.h"

#define PUBLISH_FRAME false

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_reader_node");
    ros::NodeHandle nh;

    std::string transformJsonFilePath = "/home/orangepi/robocup_ws/src/conductor/example/json/transforms.json"; // 修改为你的 JSON 文件路径

    FrameManager transformManager(transformJsonFilePath);

    std::string waypointJsonFilePath = "/home/orangepi/robocup_ws/src/conductor/example/json/rectangle_waypoint.json"; // 修改为你的 JSON 文件路径

    // 创建 WaypointManager 实例，并从 JSON 文件读取航点数据
    WaypointManager waypointManager(waypointJsonFilePath);

    ros::Rate rate(10);

#if PUBLISH_FRAME
    if (transformManager.isWorldFrameExist())
    {
        transformManager.printFrameInfoALL();
    }
    else
    {
        ROS_ERROR("World frame \"%s\" not found! exiting...", transformManager.getWorldFrameID().c_str());
        return false;
    }
#endif

    transformManager.printFrameInfoALL();
#if PUBLISH_FRAME
    transformManager.publishFrameAll();
#endif

    // 输出航点数据
    while (ros::ok())
    {
        if (!waypointManager.getCurrentTargetWaypoint().is_published)
        {
            waypointManager.printCurrentWaypoint();
#if PUBLISH_FRAME
            waypoint::Waypoint current_waypoint = waypointManager.getCurrentTargetWaypoint();
            waypoint::Waypoint world_waypoint = transformManager.getWorldWaypoint(current_waypoint);
            ROS_INFO("TRANSFORMED:");
            ROS_INFO("Index: %zu", world_waypoint.index);
            ROS_INFO("Info: %s", world_waypoint.info.c_str());
            ROS_INFO("frame_id: %s", world_waypoint.frame_id.c_str());
            ROS_INFO("Position: x= %f, y= %f, z= %f", world_waypoint.position.x, world_waypoint.position.y, world_waypoint.position.z);
            ROS_INFO("Yaw: %f", world_waypoint.yaw);
            ROS_INFO("Delay: %f", world_waypoint.delay);
            ROS_INFO("Air Speed: %f", world_waypoint.air_speed);
            ROS_INFO("Range: %f", world_waypoint.range);
            ROS_INFO(COLORED_TEXT("-----------------------------", "\033[1m"));
#endif
            waypointManager.setCurrentTargetWaypointIsPublished(true);
        }

        if (waypointManager.isWaypointDelaySatisfied())
        {
            if (!waypointManager.goToNextWaypoint())
            {
                ROS_INFO(SUCCESS("Finish print !"));
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}
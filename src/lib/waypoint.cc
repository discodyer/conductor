#include "conductor/way_point.h"
#include <ros/ros.h>
#include <rapidjson/document.h>
#include "conductor/ansi_color.h"

WaypointManager::WaypointManager(const std::string &jsonFilePath)
    : current_waypoint_index_(0), last_waypoint_time_(ros::Time::now()), is_current_waypoint_published_(false)
{
    // 打开 JSON 文件
    std::ifstream ifs(jsonFilePath);
    if (!ifs.is_open())
    {
        ROS_ERROR("Failed to open JSON file: %s", jsonFilePath.c_str());
        return;
    }

    // 读取 JSON 数据到字符串
    std::string jsonString((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ifs.close();

    // 解析 JSON 数据
    rapidjson::Document document;
    document.Parse(jsonString.c_str());

    // 检查解析是否成功
    if (!document.IsObject() || !document.HasMember("waypoints") || !document["waypoints"].IsArray())
    {
        ROS_ERROR("Invalid JSON format.");
        return;
    }

    // 获取 waypoints 数组
    std::vector<way_point::Waypoint> tempWaypoints; // 临时容器，用于暂存读取到的航点数据
    const rapidjson::Value &waypointsArray = document["waypoints"];
    for (rapidjson::SizeType i = 0; i < waypointsArray.Size(); i++)
    {
        const rapidjson::Value &waypointData = waypointsArray[i];
        if (!waypointData.IsObject())
        {
            ROS_ERROR("Invalid waypoint data.");
            continue;
        }

        // 解析航点数据
        way_point::Waypoint waypoint{i, way_point::WaypointType::kPoseWorld, {0, 0, 0}, 0, 0, 0};
        if (waypointData.HasMember("type") && waypointData["type"].IsString())
        {
            const std::string &typeStr = waypointData["type"].GetString();
            if (typeStr == "kPoseWorld")
            {
                waypoint.type = way_point::WaypointType::kPoseWorld;
            }
            else if (typeStr == "kPoseBody")
            {
                waypoint.type = way_point::WaypointType::kPoseBody;
            }
            else if (typeStr == "kSpecial")
            {
                waypoint.type = way_point::WaypointType::kSpecial;
            }
            else
            {
                ROS_ERROR("Unknown waypoint type: %s", typeStr.c_str());
                continue;
            }
        }
        else
        {
            ROS_ERROR("Waypoint type not found or not a string.");
            continue;
        }

        if (waypointData.HasMember("position") && waypointData["position"].IsObject())
        {
            const rapidjson::Value &positionData = waypointData["position"];
            if (positionData.HasMember("x") && positionData.HasMember("y") && positionData.HasMember("z") &&
                positionData["x"].IsDouble() && positionData["y"].IsDouble() && positionData["z"].IsDouble())
            {
                waypoint.position.x = positionData["x"].GetDouble();
                waypoint.position.y = positionData["y"].GetDouble();
                waypoint.position.z = positionData["z"].GetDouble();
            }
            else
            {
                ROS_ERROR("Invalid position data.");
                continue;
            }
        }
        else
        {
            ROS_ERROR("Position data not found or not an object.");
            continue;
        }

        if (waypointData.HasMember("yaw") && waypointData["yaw"].IsDouble())
        {
            waypoint.yaw = waypointData["yaw"].GetDouble();
        }
        else
        {
            ROS_ERROR("Yaw data not found or not a double.");
            continue;
        }

        if (waypointData.HasMember("delay") && waypointData["delay"].IsDouble())
        {
            waypoint.delay = waypointData["delay"].GetDouble();
        }
        else
        {
            ROS_ERROR("Delay data not found or not a double.");
            continue;
        }

        if (waypointData.HasMember("air_speed") && waypointData["air_speed"].IsDouble())
        {
            waypoint.air_speed = waypointData["air_speed"].GetDouble();
        }
        else
        {
            ROS_ERROR("Air speed data not found or not a double.");
            continue;
        }

        if (waypointData.HasMember("index") && waypointData["index"].IsInt())
        {
            waypoint.index = waypointData["index"].GetInt();
        }
        else
        {
            ROS_ERROR("Index data not found or not an integer.");
            continue;
        }

        tempWaypoints.push_back(waypoint);
    }
    // 对航点数据按照索引进行排序
    std::sort(tempWaypoints.begin(), tempWaypoints.end(),
              [](const way_point::Waypoint &a, const way_point::Waypoint &b)
              { return a.index < b.index; });

    // 将排序后的航点数据存储到 waypoints 容器中
    waypoints_ = std::move(tempWaypoints);

    ROS_INFO(SUCCESS("Loaded %zu waypoints from JSON file."), waypoints_.size());
}

bool WaypointManager::getNextWaypoint(way_point::Waypoint &waypoint)
{
    if (current_waypoint_index_ < waypoints_.size() - 1)
    {
        waypoint = waypoints_[++current_waypoint_index_];
        return true;
    }
    else
    {
        return false;
    }
}

bool WaypointManager::goToNextWaypoint()
{
    if (current_waypoint_index_ < waypoints_.size() - 1)
    {
        // 获取当前时间戳
        ros::Time current_time = ros::Time::now();

        // 判断是否满足航点延时
        if ((current_time - last_waypoint_time_).toSec() >= waypoints_[current_waypoint_index_].delay)
        {
            // 更新上一次航点发布的时间戳
            last_waypoint_time_ = current_time;

            // 转到下一个航点
            current_waypoint_index_++;

            is_current_waypoint_published_ = false;
            return true;
        }
    }
    return false;
}

way_point::Waypoint WaypointManager::getCurrentWaypoint() const
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        return waypoints_[current_waypoint_index_];
    }
    // 如果当前航点索引无效，返回一个默认的空航点
    way_point::Waypoint emptyWaypoint;
    emptyWaypoint.index = 0;
    emptyWaypoint.type = way_point::WaypointType::kPoseWorld;
    emptyWaypoint.position.x = 0.0;
    emptyWaypoint.position.y = 0.0;
    emptyWaypoint.position.z = 0.0;
    emptyWaypoint.yaw = 0.0;
    emptyWaypoint.delay = 0.0;
    emptyWaypoint.air_speed = 0.0;
    return emptyWaypoint;
}

bool WaypointManager::isWaypointDelaySatisfied() const
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        // 获取当前航点的延时
        double delay = waypoints_[current_waypoint_index_].delay;
        // 检查当前时间是否已经超过了航点的延时时间
        return (ros::Time::now() - last_waypoint_time_).toSec() >= delay;
    }
    return false;
}

way_point::WaypointType WaypointManager::getCurrentWaypointType() const
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        return waypoints_[current_waypoint_index_].type;
    }
    // 如果当前航点索引无效，返回默认类型 kPoseWorld
    return way_point::WaypointType::kPoseWorld;
}

void WaypointManager::printCurrentWaypoint() const
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        const way_point::Waypoint &waypoint = waypoints_[current_waypoint_index_];
        ROS_INFO("Current Waypoint Information:");
        ROS_INFO("Index: %zu", waypoint.index);
        ROS_INFO("Type: %d", static_cast<int>(waypoint.type));
        ROS_INFO("Position: x= %f, y= %f, z= %f", waypoint.position.x, waypoint.position.y, waypoint.position.z);
        ROS_INFO("Yaw: %f", waypoint.yaw);
        ROS_INFO("Delay: %f", waypoint.delay);
        ROS_INFO("Air Speed: %f", waypoint.air_speed);
        ROS_INFO(COLORED_TEXT("-----------------------------", "\033[1m"));
    }
    else
    {
        ROS_INFO("No more waypoints to process.");
    }
}

void WaypointManager::printCurrentWaypointLoop()
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        if (!is_current_waypoint_published_)
        {
            const way_point::Waypoint &waypoint = waypoints_[current_waypoint_index_];
            ROS_INFO("Current Waypoint Information:");
            ROS_INFO("Index: %zu", waypoint.index);
            ROS_INFO("Type: %d", static_cast<int>(waypoint.type));
            ROS_INFO("Position: x= %f, y= %f, z= %f", waypoint.position.x, waypoint.position.y, waypoint.position.z);
            ROS_INFO("Yaw: %f", waypoint.yaw);
            ROS_INFO("Delay: %f", waypoint.delay);
            ROS_INFO("Air Speed: %f", waypoint.air_speed);
            ROS_INFO(COLORED_TEXT("-----------------------------", "\033[1m"));
            is_current_waypoint_published_ = true;
        }
    }
    else
    {
        ROS_INFO("No more waypoints to process.");
    }
}
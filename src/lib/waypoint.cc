#include "conductor/waypoint.h"
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
    std::vector<waypoint::Waypoint> tempWaypoints; // 临时容器，用于暂存读取到的航点数据
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
        waypoint::Waypoint waypoint{i, waypoint::WaypointType::kPoseAbsolute, "camera_init", {0, 0, 0}, 0, 0, 0};
        if (waypointData.HasMember("type") && waypointData["type"].IsString())
        {
            const std::string &typeStr = waypointData["type"].GetString();
            if (typeStr == "PoseAbsolute")
            {
                waypoint.type = waypoint::WaypointType::kPoseAbsolute;
            }
            else if (typeStr == "PoseRelated")
            {
                waypoint.type = waypoint::WaypointType::kPoseRelated;
            }
            else if (typeStr == "Special")
            {
                waypoint.type = waypoint::WaypointType::kSpecial;
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

        if (waypointData.HasMember("frame_id") && waypointData["frame_id"].IsString())
        {
            const std::string &typeStr = waypointData["frame_id"].GetString();
            waypoint.frame_id = typeStr;
        }
        else
        {
            ROS_ERROR("frame_id not found or not a string.");
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
              [](const waypoint::Waypoint &a, const waypoint::Waypoint &b)
              { return a.index < b.index; });

    // 将排序后的航点数据存储到 waypoints 容器中
    waypoints_ = std::move(tempWaypoints);

    ROS_INFO(SUCCESS("Loaded %zu waypoints from JSON file."), waypoints_.size());
}

bool WaypointManager::getNextWaypoint(waypoint::Waypoint &waypoint)
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

waypoint::Waypoint WaypointManager::getCurrentWaypoint() const
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        return waypoints_[current_waypoint_index_];
    }
    // 如果当前航点索引无效，返回一个默认的空航点
    waypoint::Waypoint emptyWaypoint;
    emptyWaypoint.index = 0;
    emptyWaypoint.type = waypoint::WaypointType::kPoseAbsolute;
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

waypoint::WaypointType WaypointManager::getCurrentWaypointType() const
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        return waypoints_[current_waypoint_index_].type;
    }
    // 如果当前航点索引无效，返回默认类型 kPoseAbsolute
    return waypoint::WaypointType::kPoseAbsolute;
}

void WaypointManager::printCurrentWaypoint() const
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        const waypoint::Waypoint &waypoint = waypoints_[current_waypoint_index_];
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
            const waypoint::Waypoint &waypoint = waypoints_[current_waypoint_index_];
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

void WaypointManager::resetDelayTime()
{
    last_waypoint_time_ = ros::Time::now();
}

FrameManager::FrameManager(const std::string &jsonFilePath)
    : static_tf_broadcaster_(),
      tf_buffer_(),
      tf_listener_(tf_buffer_)
{
    std::vector<waypoint::FrameTransform> frameTransforms; // 临时容器

    // 打开 JSON 文件
    std::ifstream ifs(jsonFilePath);
    if (!ifs.is_open())
    {
        // 错误处理，文件打开失败
        ROS_ERROR("Failed to open JSON file: %s", jsonFilePath.c_str());
        return;
    }

    // 读取 JSON 数据到字符串
    std::string jsonString((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ifs.close();

    // 解析 JSON 数据
    rapidjson::Document document;
    document.Parse(jsonString.c_str());

    // 检查解析是否成功，并获取 transforms 数组
    if (!document.IsObject() || !document.HasMember("transforms") || !document["transforms"].IsArray())
    {
        // 错误处理，JSON 格式无效或者缺少 transforms 数组
        ROS_ERROR("Invalid JSON format.");
        return;
    }

    // 获取 transforms 数组
    const rapidjson::Value &transformsArray = document["transforms"];
    for (rapidjson::SizeType i = 0; i < transformsArray.Size(); i++)
    {
        const rapidjson::Value &transformData = transformsArray[i];
        if (!transformData.IsObject())
        {
            // 错误处理，transformData 不是对象
            ROS_ERROR("Invalid transform data.");
            continue;
        }

        // 解析坐标变换关系数据
        waypoint::FrameTransform frameTransform{};

        // 解析 source_frame_id 和 target_frame_id
        if (transformData.HasMember("source_frame_id") && transformData["source_frame_id"].IsString())
        {
            frameTransform.source_frame_id = transformData["source_frame_id"].GetString();
        }
        else
        {
            // 错误处理，source_frame_id 数据无效
            ROS_ERROR("source_frame_id not found or not a string.");
            continue;
        }

        if (transformData.HasMember("target_frame_id") && transformData["target_frame_id"].IsString())
        {
            frameTransform.target_frame_id = transformData["target_frame_id"].GetString();
        }
        else
        {
            // 错误处理，target_frame_id 数据无效
            ROS_ERROR("target_frame_id not found or not a string.");
            continue;
        }

        // 解析 translation 数据
        if (transformData.HasMember("translation") && transformData["translation"].IsObject())
        {
            const rapidjson::Value &translationData = transformData["translation"];
            if (translationData.HasMember("x") && translationData.HasMember("y") && translationData.HasMember("z") &&
                translationData["x"].IsDouble() && translationData["y"].IsDouble() && translationData["z"].IsDouble())
            {
                frameTransform.translation.setX(translationData["x"].GetDouble());
                frameTransform.translation.setY(translationData["y"].GetDouble());
                frameTransform.translation.setZ(translationData["z"].GetDouble());
            }
            else
            {
                // 错误处理，translation 数据无效
                ROS_ERROR("Invalid translation data.");
                continue;
            }
        }
        else
        {
            // 错误处理，translation 数据不存在或者不是对象
            ROS_ERROR("translation data not found or not an object.");
            continue;
        }

        // 解析 rotation 数据
        if (transformData.HasMember("rotation") && transformData["rotation"].IsObject())
        {
            const rapidjson::Value &orientationData = transformData["rotation"];
            if (orientationData.HasMember("yaw") && orientationData["yaw"].IsDouble())
            {
                frameTransform.rotation.setRPY(0, 0, orientationData["yaw"].GetDouble());
                frameTransform.rotation.normalize();
            }
            else
            {
                // 错误处理，rotation 数据无效
                ROS_ERROR("Yaw data not found or not a double.");
                continue;
            }
        }
        else
        {
            // 错误处理，rotation 数据不存在或者不是对象
            ROS_ERROR("rotation data not found or not an object.");
            continue;
        }

        // 将解析的坐标变换关系添加到 vector 中
        frameTransforms.push_back(frameTransform);
    }
    frameTransforms_ = std::move(frameTransforms);
    ROS_INFO(SUCCESS("Loaded %zu transform frames from JSON file."), frameTransforms_.size());
}

bool FrameManager::isWorldFrameExist() const
{
    if (!frameTransforms_.empty())
    {
        // 获取第一个 frameTransform 的 target_frame_id
        const std::string &world_frame_id = frameTransforms_[0].target_frame_id;
        return isFrameExist(world_frame_id);
    }
    // 如果 frameTransforms_ 为空，直接返回 false
    return false;
}

bool FrameManager::isFrameExist(const std::string &frame_id) const
{
    return tf_buffer_._frameExists(frame_id);
}
bool FrameManager::isTargetExist(const waypoint::FrameTransform &frame) const
{
    return isFrameExist(frame.target_frame_id);
}

void FrameManager::publishFrame(const waypoint::FrameTransform &frame)
{
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = frame.target_frame_id;
    transform_stamped.child_frame_id = frame.source_frame_id;
    transform_stamped.transform.translation.x = frame.translation.getX();
    transform_stamped.transform.translation.y = frame.translation.getY();
    transform_stamped.transform.translation.z = frame.translation.getZ();
    transform_stamped.transform.rotation.x = frame.rotation.getX();
    transform_stamped.transform.rotation.y = frame.rotation.getY();
    transform_stamped.transform.rotation.z = frame.rotation.getZ();
    transform_stamped.transform.rotation.w = frame.rotation.getW();
    static_tf_broadcaster_.sendTransform(transform_stamped);

    double yaw, pitch, roll;
    tf2::Matrix3x3(frame.rotation).getEulerYPR(yaw, pitch, roll);
    ROS_INFO("Published new Transform:");
    ROS_INFO("%s <-- %s", frame.target_frame_id.c_str(), frame.source_frame_id.c_str());
    ROS_INFO("x: %f", frame.translation.getX());
    ROS_INFO("y: %f", frame.translation.getY());
    ROS_INFO("z: %f", frame.translation.getZ());
    ROS_INFO("yaw: %f", yaw);
    ROS_INFO("------------------------");
}

void FrameManager::publishFrameAll()
{
    for(const auto& frame : frameTransforms_)
    {
        // 使用 static_tf_broadcaster_ 发布坐标关系
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = frame.target_frame_id;
        transform_stamped.child_frame_id = frame.source_frame_id;
        transform_stamped.transform.translation.x = frame.translation.x();
        transform_stamped.transform.translation.y = frame.translation.y();
        transform_stamped.transform.translation.z = frame.translation.z();
        transform_stamped.transform.rotation.x = frame.rotation.x();
        transform_stamped.transform.rotation.y = frame.rotation.y();
        transform_stamped.transform.rotation.z = frame.rotation.z();
        transform_stamped.transform.rotation.w = frame.rotation.w();

        static_tf_broadcaster_.sendTransform(transform_stamped);
    }
}
#include "conductor/drone_control.h"

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "circle_flight");
    ros::NodeHandle nh;

    // 设置循环频率
    ros::Rate rate(20.0);

    // 指定围绕的坐标点
    float center_x = 1.0;
    float center_y = 0.0;
    float center_z = 1.0;
    float radius = 1.0;
    float angular_speed = 0.5;  // 角速度，单位：rad/s

    // 等待连接到飞控
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // 解锁无人机
    if (arm_drone(nh) != 0)
    {
        return -1;
    }

    // 设置为GUIDED模式
    if (set_guided(nh) != 0)
    {
        return -1;
    }

    // 起飞
    float takeoff_height = 1.0;
    if (takeoff(nh, takeoff_height) != 0)
    {
        return -1;
    }

    // 等待起飞完成
    for (int i = 0; i < 1500; i++) // 15s, wait for takeoff
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // 开始围绕坐标点飞行
    while (ros::ok())
    {
        // 计算飞机当前位置与目标位置之间的距离和角度
        float current_x = current_pose.pose.position.x;
        float current_y = current_pose.pose.position.y;
        float dx = current_x - center_x;
        float dy = current_y - center_y;
        float distance = sqrt(dx * dx + dy * dy);
        float target_yaw = atan2(dy, dx);

        // 计算飞机目标位置
        float target_x = center_x + radius * cos(target_yaw);
        float target_y = center_y + radius * sin(target_yaw);
        float target_z = center_z;

        // 设置飞机目标位置
        set_pose_local(target_x, target_y, target_z, target_yaw);

        // 判断是否到达目标位置
        if (distance < 0.1)  // 到达目标位置的阈值，可以根据需要调整
        {
            break;  // 停止飞行
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 降落
    if (land(nh) != 0)
    {
        return -1;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

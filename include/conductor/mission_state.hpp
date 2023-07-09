#ifndef MISSION_STATE_HPP
#define MISSION_STATE_HPP

// 安全的SIGINT处理函数,能够紧急自动降落
void safeSigintHandler(int sig);

// 飞行任务状态
enum MissionState
{
    prearm,
    arm,
    takeoff,
    pose,
    land
};

#endif // MISSION_STATE_HPP
// example.cpp
#include <iostream>
#include "DualSteeredWheelchair.h"
#include "UdpComm.h"

#include <nlohmann/json.hpp>
// 使用nlohmann json库的命名空间
using json = nlohmann::json;

uint8_t updata_key = 0;
double speed_vx = 0;
double speed_vy = 0;
double speed_rz = 0;
// JSON解析回调函数
void onMessageReceived(const std::string &message, const std::string &ip, int port)
{

    try
    {
        uint8_t key = 0;
        // 解析JSON数据
        json jsonData = json::parse(message);
        // std::cout << "解析成功: " << jsonData.dump(4) << std::endl;

        // 根据业务需求处理不同的JSON格式
        if (jsonData.contains("speed"))
        {
            json &data = jsonData["speed"];
            speed_vx = data[0];
            speed_vy = data[1];
            speed_rz = data[2];
            key = 1;
        }

        updata_key = key;
    }
    catch (const json::parse_error &e)
    {
        std::cerr << "JSON解析错误: " << e.what() << std::endl;
        std::cerr << "错误数据: " << message << std::endl;
    }
}

int main()
{
    // 创建UDP通信对象，监听5353端口
    UdpComm udpComm(5353, onMessageReceived);
    // 启动监听
    if (!udpComm.start())
    {
        std::cerr << "启动UDP通信失败" << std::endl;
        return -1;
    }

    double wheel_track = 0.75; // 60 cm between wheels
    DualSteeredWheelchair chair(wheel_track, 0.005);

    // Desired body motion: move forward at 0.5 m/s, no lateral motion, no rotation
    while (1)
    {
        auto cmd = chair.computeWheelCommands(speed_vx, speed_vy, speed_rz);

        std::cout << "vl = " << cmd.vl << " m/s\n";
        std::cout << "vr = " << cmd.vr << " m/s\n";
        std::cout << "thl = " << cmd.thl << " rad (" << cmd.thl * 180.0 / M_PI << " deg)\n";
        std::cout << "thr = " << cmd.thr << " rad (" << cmd.thr * 180.0 / M_PI << " deg)\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
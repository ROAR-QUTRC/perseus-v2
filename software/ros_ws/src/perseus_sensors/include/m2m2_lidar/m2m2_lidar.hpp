#pragma once

#include <cstdint>
#include <string>

class M2M2Lidar
{
    M2M2Lidar(std::string address, uint16_t port);

    // delete move + copy semantics
    M2M2Lidar(const M2M2Lidar& other) = delete;
    M2M2Lidar(M2M2Lidar&& other) = delete;
    M2M2Lidar& operator=(const M2M2Lidar& other) = delete;
    M2M2Lidar& operator=(M2M2Lidar&& other) = delete;
};
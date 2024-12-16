#pragma once
#include <librealsense2/rs.hpp>

struct PoseData{
    float x, y, z;
    float rotation_x, rotation_y, rotation_z, rotation_w;
    float x_vel, y_vel, z_vel;
};

PoseData get_pose_data(rs2::pipeline& pipe);

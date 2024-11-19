#include "../../include/common/PoseData.h"
#include <iostream>

PoseData get_pose_data(rs2::pipeline& pipe) {
    PoseData pd;

    // Attempt to get the next set of frames from the camera
    rs2::frameset frames;

    if (pipe.try_wait_for_frames(&frames, 0)) {
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);

        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Save pose data
        pd.x = pose_data.translation.x;
        pd.y = pose_data.translation.y;
        pd.z = pose_data.translation.z;
        pd.rotation_x = pose_data.rotation.x;
        pd.rotation_y = pose_data.rotation.y;
        pd.rotation_z = pose_data.rotation.z;
        pd.rotation_w = pose_data.rotation.w;
        pd.x_vel = pose_data.velocity.x;
        pd.y_vel = pose_data.velocity.y;
        pd.z_vel = pose_data.velocity.z;
    }
    return pd;
}

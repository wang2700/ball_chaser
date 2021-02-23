#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // ROS_INFO_STREAM("Command robot to move at velocity L.x:" + std::to_string(lin_x) 
    //                 + "  A.z: " + std::to_string(ang_z));

    //Request the desired velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and read the image data
void process_image_callback(const sensor_msgs::Image img) 
{
    int white_pixel = 255;
    bool found_ball = false;
    int ball_location_col = 0;
    const int right_limit = 250;
    const int left_limit = img.width - right_limit;
    
    for (int i = 0; i < img.height * img.step; i += 3) {
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
            found_ball = true;
            ball_location_col = (i/3) % img.width;             
            break;
        }
    }

    if (found_ball) {
        ROS_INFO_STREAM("Ball found at column: " + std::to_string(ball_location_col));
        if (ball_location_col < right_limit) {
            ROS_INFO_STREAM("Turn Left");
            drive_robot(0, 0.1);
        }
        else if (ball_location_col > left_limit) {
            ROS_INFO_STREAM("Turn Right");
            drive_robot(0, -0.1);
        }
        else {
            drive_robot(0.3, 0);
        }
    }
    else {
        // drive_robot(0, 0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
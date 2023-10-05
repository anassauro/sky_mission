#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string.h>
#include <utility>

using namespace ros;

#define ARMING 0
#define TAKEOFF 1
#define CENTRALIZE 2
#define LAND 3
#define TRAVEL 4
#define WAIT 5
#define RTL 6

//global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
std::pair<int, int> center;
int height, width;
bool rtl = false;


//callback functions

void rtl_callback(const std_msgs::Bool::ConstPtr& msg) {
    rtl = *msg;
}

void state_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void ahrs_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //get drone altitude
    current_pose = *msg;
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg) {
    //get image
    height = msg->height;
    width = msg->width;
}


void bounding_box_callback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    //get bounding box
    center.first = (msg->data[0] + msg->data[2]) / 2;
    center.second = (msg->data[1] + msg->data[3]) / 2;
}


void centralize(int width, int height, int cX, int cY, Publisher* pub, float* error, int z_offset = -1) {
    //width e height são as dimensões da imagem
    //cX e cY são as coordenadas do centro da figura, em que queremos centralizar
    //z_offset é a altura em que o drone vai ficar ao centralizar, -1 -> indica que a altura não será alterada

    float kp = 0.5, ki = 0.005, kd = 0.05; //constantes para PID
    float error_x = width / 2 - cX, error_y = height / 2 - cY, error_z = z_offset - current_pose.pose.position.z;
    float last_error_x = 0, last_error_y = 0, last_error_z = 0;
    float I_x = 0, I_y = 0, I_z = 0;
    int pot = 50;

    //set drone velocity 
    geometry_msgs::Twist velocity;
    velocity.linear.x = kp * error_x + ki * I_x + kd * (error_x - last_error_x);
    velocity.linear.y = kp * error_y + ki * I_y + kd * (error_y - last_error_y);

    //error vector
    error[0] = error_x;
    error[1] = error_y;

    //udpate last errors
    last_error_x = error_x;
    last_error_y = error_y;

    //update integral term
    I_x += error_x;
    I_y += error_y;

    //send velocity to drone
    pub->publish(velocity);
}

int main(int argc, char** argv) {
    //iniciando node
    init(argc, argv, "centralize");
    NodeHandle nh;

    int state = ARMING;
    bool keep_running = true;

    //publisers
    Publisher pub_velocity = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    Publisher pub_local_pos = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

    //services
    ServiceClient client_arming = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming", 1);
    ServiceClient client_takeoff = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff", 1);
    ServiceClient client_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode", 1);

    //subscribers
    Subscriber sub_pose = nh.subscribe("/mavros/local_position/pose", 10, ahrs_callback);
    Subscriber sub_state = nh.subscribe("mavros/state", 10, state_callback);
    Subscriber sub_image = nh.subscribe("/sky_vision/down_cam/img_result", 10, image_callback);
    Subscriber sub_bounding_box = nh.subscribe("/sky_vision/down_cam/pad/bounding_box", 10, bounding_box_callback);
    Subscriber sub_rtl = nh.subscribe("/stage_4/rtl", 10, rtl_callback);

    Rate loop_rate(60);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ROS_INFO("waiting for FCU connection");
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ok() && keep_running) {

        //spin ros master 
        spinOnce();
        loop_rate.sleep();

        mavros_msgs::SetMode mode;
        switch (state)
        {
        case ARMING: {
            //arming
            ROS_INFO("arming");
            mode.request.custom_mode = "GUIDED"; //ardupilot GUIDED mode
            client_mode.call(mode);
            ROS_INFO("mode sent: %d", mode.response.mode_sent);
            ROS_INFO("mode: %s", current_state.mode.c_str());
            if (mode.response.mode_sent && current_state.mode == "GUIDED") {
                mavros_msgs::CommandBool arm;
                arm.request.value = true;
                client_arming.call(arm);
                if (arm.response.success) state = TAKEOFF;
            }
            break;
        }

        case TAKEOFF: {
            //takeoff
            ROS_INFO("takeoff");
            ROS_INFO("altitude: %f", current_pose.pose.position.z);
            mavros_msgs::CommandTOL takeoff;
            takeoff.request.min_pitch = 0;
            takeoff.request.yaw = 0;
            takeoff.request.latitude = 0;
            takeoff.request.longitude = 0;
            takeoff.request.altitude = 2;

            client_takeoff.call(takeoff);
            if (takeoff.response.success && current_pose.pose.position.z == takeoff.request.altitude) state = TRAVEL;
            break;
        }
        case TRAVEL: {
            //travel
            ROS_INFO("travel");
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = -1.7;
            pose.pose.position.y = -1.2;
            pose.pose.position.z = 2;
            pub_local_pos.publish(pose);
            state = CENTRALIZE;
            break;
        }
        case CENTRALIZE: {
            //centralize
            ROS_INFO("centralize");
            float error[2] = { 1000, 1000 };
            ROS_INFO("width: %d, height: %d", width, height);
            ROS_INFO("center: (%d, %d)", center.first, center.second);
            while (abs(error[0]) > 5 || abs(error[1]) > 5) {
                //use sky_vision
                ROS_INFO("trying to centralize");
                centralize(width, height, center.first, center.second, &pub_velocity, error);
                ROS_INFO("error_x: %f, error_y: %f", error[0], error[1]);
            }

            state = LAND;
            break;
        }

        case LAND: {
            //land
            ROS_INFO("land");
            mode.request.custom_mode = "LAND"; //ardupilot LAND mode
            client_mode.call(mode);
            if (mode.response.mode_sent) {
                ROS_INFO("landed");
                keep_running = false;
            }
            break;
        }

        case WAIT: {
            ROS_INFO("waiting for takeoff");
            if (rtl) {
                mavros_msgs::CommandTOL takeoff;
                takeoff.request.min_pitch = 0;
                takeoff.request.yaw = 0;
                takeoff.request.latitude = 0;
                takeoff.request.longitude = 0;
                takeoff.request.altitude = 2;

                client_takeoff.call(takeoff);
                if (takeoff.response.success && current_pose.pose.position.z == takeoff.request.altitude) state = RTL;
            }
        }

        case RTL: {
            ROS_INFO("RTL");
            mode.request.custom_mode = "RTL"; //ardupilot RTL mode
            client_mode.call(mode);
            if (mode.response.mode_sent) {
                ROS_INFO("RTL");
                keep_running = false;
            }
            break;
        }

        default: {
            ROS_INFO("nothing");
            if (current_state.mode != "LAND") state = LAND;
            break;
        }
        }

    }

    return 0;
}
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string.h>

using namespace ros;

#define ARMING 0
#define TAKEOFF 1
#define CENTRALIZE 2
#define LAND 3


//global variables
float altitude; // get z height

//callback functions
mavros_msgs::State current_state;
void state_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void ahrs_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //get drone altitude
    altitude = msg->pose.position.z;
}

float* centralize(int width, int height, int cX, int cY, Publisher* pub, int z_offset = -1) {
    //width e height são as dimensões da imagem
    //cX e cY são as coordenadas do centro da figura, em que queremos centralizar
    //z_offset é a altura em que o drone vai ficar ao centralizar, -1 -> indica que a altura não será alterada

    float kp = 0.5, ki = 0.005, kd = 0.05; //constantes para PID
    float error_x = width / 2 - cX, error_y = height / 2 - cY, error_z = z_offset - altitude;
    float last_error_x = 0, last_error_y = 0, last_error_z = 0;
    float I_x = 0, I_y = 0, I_z = 0;
    int pot = 50;

    //set drone velocity 
    geometry_msgs::Twist velocity;
    velocity.linear.x = kp * error_x + ki * I_x + kd * (error_x - last_error_x);
    velocity.linear.y = kp * error_y + ki * I_y + kd * (error_y - last_error_y);

    //udpate last errors
    last_error_x = error_x;
    last_error_y = error_y;

    //update integral term
    I_x += error_x;
    I_y += error_y;

    //send velocity to drone
    pub->publish(velocity);

    float error[2] = { error_x, error_y };

    return error;
}

int main(int argc, char** argv) {
    //iniciando node
    init(argc, argv, "centralize");
    NodeHandle nh;

    int state = ARMING;

    //publisers
    Publisher pub_velocity = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    ServiceClient client_arming = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming", 1);
    ServiceClient client_takeoff = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff", 1);
    ServiceClient client_mode = nh.serviceClient<mavros_msgs::SetMode>("/set_mode", 1);

    //subscribers
    Subscriber sub_pose = nh.subscribe("/mavros/local_position/pose", 1, ahrs_callback);
    ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);

    Rate loop_rate(60);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ROS_INFO("waiting for FCU connection");
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ok()) {

        //spin ros master 
        spinOnce();
        loop_rate.sleep();

        std_msgs::String msg;
        mavros_msgs::SetMode mode;
        switch (state)
        {
        case ARMING: {
            //arming
            msg.data = "arming";
            ROS_INFO("%s", msg.data.c_str());
            mode.request.custom_mode = "GUIDED"; //ardupilot GUIDED mode
            client_mode.call(mode);
            ROS_INFO("mode sent: %d", mode.response.mode_sent);
            if (mode.response.mode_sent) {
                mavros_msgs::CommandBool arm;
                arm.request.value = true;
                client_arming.call(arm);
                if (arm.response.success) state = TAKEOFF;
            }
            break;
        }

        case TAKEOFF: {
            //takeoff
            msg.data = "takeoff";
            ROS_INFO("%s", msg.data.c_str());
            mavros_msgs::CommandTOL takeoff;
            takeoff.request.altitude = 2;
            client_takeoff.call(takeoff);
            if (takeoff.response.success) state = CENTRALIZE;
            break;
        }

        case CENTRALIZE: {
            //centralize
            msg.data = "centralize";
            ROS_INFO("%s", msg.data.c_str());
            float* error;
            while (abs(error[0]) > 5 || abs(error[1]) > 5) {
                //use sky_vision
                error = centralize(640, 480, 320, 240, &pub_velocity);
            }

            state = LAND;
            break;
        }

        case LAND: {
            //land
            msg.data = "land";
            ROS_INFO("%s", msg.data.c_str());
            mode.request.custom_mode = "LAND"; //ardupilot LAND mode
            client_mode.call(mode);
            if (mode.response.mode_sent) ROS_INFO("landed");
            break;
        }

        default: {
            msg.data = "nothing";
            ROS_INFO("%s", msg.data.c_str());
            state = LAND;
            break;
        }
        }

    }

    return 0;
}
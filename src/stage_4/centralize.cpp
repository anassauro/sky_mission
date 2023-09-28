#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <string.h>

using namespace ros;

//global variables
float altitude; // get z height

void ahrs_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //get drone altitude
    altitude = msg->pose.position.z;
}

int* centralize(int width, int height, int cX, int cY, int z_offset = -1, Publisher* pub) {
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

    int error[2] = { error_x, error_y };

    return error;
}

int main(int argc, char** argv) {
    //iniciando node
    init(argc, argv, "centralize");
    NodeHandle nh;

    String state = "nothing";

    //publisers
    Publisher pub_velocity = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    Publisher pub_arming = nh.advertise<mavros_msgs::CommandBool>("/mavros/cmd/arming", 1);
    Publisher pub_takeoff = nh.advertise<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff", 1);
    Publisher pub_land = nh.advertise<mavros_msgs::CommandTOL>("/mavros/cmd/land", 1);

    //subscribers
    Subscriber sub_pose = nh.subscribe("/mavros/local_position/pose", 1, ahrs_callback);

    Rate loop_rate(60);
    while (ok()) {

        //spin ros master 
        spinOnce();
        loop_rate.sleep();

        switch (state)
        {
        case "nothing":
            cout << "waiting for start" << endl;
            cout << "chose a state: " << endl;
            while (state == "nothing") cin >> state;
            break;
        case "arming":
            //arming
            mavros_msgs::CommandBool arm;
            arm.value = true;
            pub_arming.publish(arm);
            state = "takeoff";
            break;

        case "takeoff":
            //takeoff
            mavros_msgs::CommandTOL takeoff;
            takeoff.altitude = 2;
            pub_takeoff.publish(takeoff);
            state = "centralize";
            break;

        case "centralize":
            //centralize
            while (abs(error_x) > 5 || abs(error_y) > 5)) {
                //use sky_vision
                int error[2];
                error = centralize(640, 480, 320, 240, &pub);
            }

            state = "land";
            break;

        case "land":
            //land
            mavros_msgs::CommandTOL land;
            land.altitude = 0;
            pub_land.publish(land);
            state = "nothing";
            break;

        default:
            cout << "invalid state" << endl;
            state = "land";
            break;
        }

    }

    return 0;
}
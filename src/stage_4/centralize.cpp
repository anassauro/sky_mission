#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

using namespace ros;

//global variables
float altitude; // get z height

void ahrs_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //get drone altitude
    altitude = msg->pose.position.z;
}

void centralize(int width, int height, int cX, int cY, int z_offset = -1, Publisher* pub) {
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
    return 0;
}

int main(int argc, char** argv) {
    //iniciando node
    init(argc, argv, "centralize");
    nodeHandle nh, n;

    Publisher pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    Subscriber sub = n.subscribe("/mavros/local_position/pose", 1, ahrs_callback);

    Rate loop_rate(60);
    while (ok()) {
        centralize(640, 480, 320, 240, -1, &pub);

        spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
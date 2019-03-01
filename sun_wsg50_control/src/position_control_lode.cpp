#include "ros/ros.h"
#include "sun_wsg50_common/Status.h"
#include "std_msgs/Float32.h"


ros::Publisher pub_vel;

std_msgs::Float32 goal_speed_msg;

using namespace std;

float p_gain = 15.0;
float position_rif = 100.0;

void statusCallback( const sun_wsg50_common::StatusConstPtr & msg ){

    goal_speed_msg.data = p_gain * ( position_rif - msg->width );

    pub_vel.publish(goal_speed_msg);

}

void positionCallback( const std_msgs::Float32ConstPtr & msg ){

    if(msg->data > 38.0){
        position_rif = -38.0+100.0;
        cout << "LIMIT!!!" << endl;
    }
	 else if(msg->data < 0.0){
		  position_rif = 0.0+100.0;
        cout << "LIMIT!!!" << endl;
	}
    else
        position_rif = -msg->data + 100.0;

}


int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "position_control");
    
    ros::NodeHandle nh_public;

    ros::Subscriber sub_status = nh_public.subscribe("/status", 1, statusCallback);

    ros::Subscriber sub_pos_rif = nh_public.subscribe("/position_cmd", 1, positionCallback);
    
    pub_vel = nh_public.advertise<std_msgs::Float32>("/goal_speed", 1);

    ros::spin();

    return 0;
}

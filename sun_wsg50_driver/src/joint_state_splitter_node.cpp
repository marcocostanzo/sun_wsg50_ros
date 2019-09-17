#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

sensor_msgs::JointState out;
ros::Publisher pub;
double scale = 1.0;

void subCallback( const sensor_msgs::JointState::ConstPtr& msg )
{
    
    out.header = msg->header;
    out.position[0] = -scale*msg->position[0]/2.0;
    out.position[1] = -out.position[0];
    out.velocity[0] = -scale*msg->velocity[0]/2.0;
    out.velocity[1] = -out.velocity[0];

    pub.publish(out);

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "joint_state_splitter");
    
    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");

    nh_private.param( "scale", scale, 1.0 );

    ros::Subscriber sub = nh.subscribe("/wsg_50_driver/joint_states", 1000, subCallback);

    pub = nh.advertise<sensor_msgs::JointState>("/wsg_50_driver/joint_states/splitted", 1000);
    
    
    out.name.push_back("finger_joint1");
    out.name.push_back("finger_joint2");
    out.position.resize(2);
    out.velocity.resize(2);
    out.effort.resize(2);
    
    ros::spin();

    return 0;
}
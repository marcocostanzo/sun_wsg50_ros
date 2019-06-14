/*
    ROS node to monitoring gripper width

    Copyright 2018-2019 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ros/ros.h"
#include "sun_ros_msgs/Float64Stamped.h"

using namespace std;

//Global ROS vars
ros::Publisher pub_speed;

//Global Vars
double width_min, width_max, width;

void width_cb(const sun_ros_msgs::Float64Stamped::ConstPtr& width_msg)
{
    width = width_msg->data;
}

void monitor_cb(const sun_ros_msgs::Float64Stamped::ConstPtr& speed_in)
{

    sun_ros_msgs::Float64Stamped speed_out;
    if( (width >= width_max && speed_in->data > 0.0) || (width <= width_min && speed_in->data < 0.0) ){
        speed_out.data = 0.0;
        ROS_WARN_THROTTLE( 10 ,"GRIPPER MONITORING: INVALID WIDTH");
    } else {
        speed_out.data = speed_in->data;
    }

    //Check invalid inputs
    if( isinf(speed_out.data) || isnan(speed_out.data) )
    {
        ROS_ERROR_THROTTLE( 0.5 ,"GRIPPER MONITORING: INF OR NAN: %lf", speed_out.data );
        speed_out.data = 0.0;
    }

    speed_out.header = speed_in->header;
    pub_speed.publish(speed_out);

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "status_monitoring");
    
    ros::NodeHandle nh_public = ros::NodeHandle();
    ros::NodeHandle nh_private("~");

    //Params
    nh_private.param("width_min" , width_min, 0.018 );
    nh_private.param("width_max" , width_max, 0.108 );

    string topic_goal_speed_in("");
    nh_private.param("topic_goal_speed_in" , topic_goal_speed_in, string("goal_speed/unmonitored") );
    string topic_goal_speed_out("");
    nh_private.param("topic_goal_speed_out" , topic_goal_speed_out, string("goal_speed") );
    string topic_width("");
    nh_private.param("topic_width" , topic_width, string("width") );

    //Pub/Sub
    pub_speed = nh_public.advertise<sun_ros_msgs::Float64Stamped>(topic_goal_speed_out, 1);
    ros::Subscriber sub_width = nh_public.subscribe(topic_width, 1, width_cb);
    ros::Subscriber sub_speed = nh_public.subscribe(topic_goal_speed_in, 1, monitor_cb);

    ros::spin();

    return 0;
}
/*
    ROS node to filter float64 stamped

    Copyright 2018 Università della Campania Luigi Vanvitelli

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
#include "sun_systems_lib/TF/TF_FIRST_ORDER_FILTER.h"

using namespace std;

ros::Publisher pubFloatFilter;
sun_ros_msgs::Float64Stamped msgFloatFilter;

double fr;
double fm;

//==========TOPICs CALLBKs=========//
void readRef( const sun_ros_msgs::Float64Stamped::ConstPtr& msg  ){

    fr = msg->data;
	
}

void readMeasure( const sun_ros_msgs::Float64Stamped::ConstPtr& msg  ){

    fm = msg->data;
	
}

//====================================//


int main(int argc, char *argv[])
{

    ros::init(argc,argv,"force_ref_filt");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    /**** PARAMS ****/
    string str_in_topic = string("");
    nh_private.param("in_topic" , str_in_topic, string("/tactile") );
    string str_measure_topic = string("");
    nh_private.param("measure_topic" , str_measure_topic, string("/tactile") );
    string str_out_topic = string("");
    nh_private.param("out_topic" , str_out_topic, str_in_topic + string("/filter") );
    double cut_freq;
    nh_private.param("cut_freq" , cut_freq, 20.0 );
    double Hz;
    nh_private.param("rate" , Hz, 500.0 );
    double max_force;
    nh_private.param("max_force" , max_force, 15.0 );
	/************************************/

    /******INIT ROS MSGS**********/
	/********************/

    /*******INIT ROS PUB**********/
	//Float_filter pub
	pubFloatFilter = nh_public.advertise<sun_ros_msgs::Float64Stamped>( str_out_topic, 1);
    /***************************/

    /*******INIT ROS SUB**********/
	ros::Subscriber subRef = nh_public.subscribe( str_in_topic , 1, readRef);
    ros::Subscriber subMeasure = nh_public.subscribe( str_measure_topic , 1, readMeasure);
    /***************************/

    /******INIT FILTER************/
    sun::TF_FIRST_ORDER_FILTER filter(cut_freq, 1.0/Hz);
    /***************************/	

    /*============LOOP==============*/
    ros::Rate loop_rate(Hz);
	while(ros::ok()){

        double fr_star = fr;
        if(fabs(fm)>max_force)
        {
            fr_star = max_force;
        }

        msgFloatFilter.data = filter.apply( fr_star );
        msgFloatFilter.header.stamp = ros::Time::now();
		pubFloatFilter.publish( msgFloatFilter );
		loop_rate.sleep();
      	ros::spinOnce();
			
	}
    
    /*==============================*/

    return 0;
}
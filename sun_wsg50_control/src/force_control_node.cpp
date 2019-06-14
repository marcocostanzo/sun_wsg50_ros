/*
    ROS node to control the normal force

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
#include <signal.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sun_ros_msgs/Float64Stamped.h>
#include "std_srvs/SetBool.h"

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
/*===============================*/

#define HEADER_PRINT BOLDYELLOW "[Force Controller]: " CRESET

using namespace std;

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}


ros::NodeHandle * nh_public;
ros::Publisher velPub;
ros::Subscriber force_sub;
ros::Subscriber force_command_sub;

string topic_force_command_str("");
string topic_measure_str("");
string topic_measure_type_str("");

double fz = 0.0;
double fr = 0.0;
double max_force;
double control_gain;
double stiff_1;
double stiff_2;
bool b_linear_model;
sun_ros_msgs::Float64Stamped velMsg;

double inv_stiff(double f){
    if(b_linear_model){
        return f/stiff_1;
    } else{
        return sqrt( fabs(f)/stiff_2 );
    }
}

void applyControl(){
    double dx = inv_stiff(fz);
    double dx_r = inv_stiff(fr);
    velMsg.data = -control_gain*(dx_r-fabs(dx));
    velMsg.data = velMsg.data*1.0E3; // m/s  to mm/s
    velMsg.header.stamp = ros::Time::now();
    velPub.publish(velMsg);
}

void readForceWrenchStamped(const geometry_msgs::WrenchStamped::ConstPtr& forceMsg){
	fz = forceMsg->wrench.force.z;
	applyControl();
}
void readForceWrench(const geometry_msgs::Wrench::ConstPtr& forceMsg){
	fz = forceMsg->force.z;
	applyControl();
}
void readForceFloat64(const std_msgs::Float64::ConstPtr& forceMsg){
	fz = forceMsg->data;
	applyControl();
}
void readForceFloat64Stamped(const sun_ros_msgs::Float64Stamped::ConstPtr& forceMsg){
	fz = forceMsg->data;
	applyControl();
}

void readCommand(const sun_ros_msgs::Float64Stamped::ConstPtr& forceMsg){

	fr = fabs(forceMsg->data);
    //saturation
    if(fr>max_force){
        fr = max_force;
        //cout << HEADER_PRINT << BOLDRED << "MAX FORCE!" << max_force << CRESET << endl;
    }
}

void sendZeroVel(){
    velMsg.data = 0.0;
    velMsg.header.stamp = ros::Time::now();
    velPub.publish(velMsg);
    velPub.publish(velMsg);
    velPub.publish(velMsg);
    velPub.publish(velMsg);
}

void stopSubscribers(){
    force_sub.shutdown();
    force_command_sub.shutdown();
}

void startSubscribers(){
    switch (str2int(topic_measure_type_str.c_str()))
    {
        case str2int("Float64"):{
            force_sub = nh_public->subscribe(topic_measure_str, 1, readForceFloat64);
            break;
        }

        case str2int("Float64Stamped"):{
            force_sub = nh_public->subscribe(topic_measure_str, 1, readForceFloat64Stamped);
            break;
        }

        case str2int("WrenchStamped"):{
            force_sub = nh_public->subscribe(topic_measure_str, 1, readForceWrenchStamped);
            break;
        }

        case str2int("Wrench"):{
            force_sub = nh_public->subscribe(topic_measure_str, 1, readForceWrench);
            break;
        }
            
        default:{
            cout << HEADER_PRINT BOLDRED "Invalid topic_measure_type_str = " << YELLOW << topic_measure_type_str << CRESET << endl;
            exit(-1);
        }
    }
	force_command_sub = nh_public->subscribe(topic_force_command_str, 1, readCommand);
}

/*Pause callback*/
bool running = true;
bool setRunning_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

    if(req.data){

        if(!running){
            startSubscribers();
		    cout << HEADER_PRINT GREEN "STARTED!" CRESET << endl;
        }
        running = true;

	} else{
        stopSubscribers();
        sendZeroVel();
        running = false;
		cout << HEADER_PRINT YELLOW "PAUSED!" CRESET << endl;  
    }

    res.success = true;
	return true;
}


void intHandler(int dummy) {
    sendZeroVel();
    ros::shutdown();
}


int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "force_controller");

    nh_public = new ros::NodeHandle();
    ros::NodeHandle nh_private("~");
	 
    nh_private.param("topic_measure" , topic_measure_str, string("grasp_force") );
    nh_private.param("topic_measure_type" , topic_measure_type_str, string("Float64Stamped") );
    
    nh_private.param("topic_force_command" , topic_force_command_str, string("command_force") );
	string topic_goal_speed("");
    nh_private.param("topic_goal_speed" , topic_goal_speed, string("goal_speed") );

    string set_running_service_str("");
    nh_private.param("set_running_service" , set_running_service_str, string("set_running") );
    nh_private.param("start_running" , running, false );

    nh_private.param("control_gain" , control_gain, 1.0 );
    nh_private.param("max_force" , max_force, 20.0 );
    nh_private.param("stiff_1" , stiff_1, 5000.0 );
    nh_private.param("stiff_2" , stiff_2, 5.0839*1.0E6 );
    nh_private.param("use_linear_model" , b_linear_model, false );

	 // Publisher
     if(running){
        startSubscribers();
     }
	 velPub = nh_public->advertise<sun_ros_msgs::Float64Stamped>( topic_goal_speed,1);

    ros::ServiceServer servicePause = nh_public->advertiseService(set_running_service_str, setRunning_callbk);

    control_gain = fabs(control_gain);
    max_force = fabs(max_force);

    if(stiff_1 <= 0 || stiff_2 <= 0){
        cout << HEADER_PRINT BOLDRED "Both stiff have to be > 0 | Fatal ERROR" CRESET << endl;
        exit(-1); 
    }

    signal(SIGINT, intHandler);

    ros::spin();

	sendZeroVel();

    delete nh_public;

    return 0;
}

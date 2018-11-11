/*
    ROS node to control the normal force

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
#include <signal.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
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
float max_force = 20.0;
float control_gain = 1.0;
std_msgs::Float32 velMsg;

void applyControl(){
    velMsg.data = -control_gain*(fr-fabs(fz));
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

void readCommand(const std_msgs::Float64::ConstPtr& forceMsg){

	fr = fabs(forceMsg->data);
    //saturation
    if(fr>max_force){
        fr = max_force;
        //cout << HEADER_PRINT << BOLDRED << "MAX FORCE!" << max_force << CRESET << endl;
    }
}

void sendZeroVel(){
    velMsg.data = 0.0;
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
bool paused = true;
bool pause_callbk(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){

	if(req.data){

        stopSubscribers();
        sendZeroVel();
		cout << HEADER_PRINT YELLOW "PAUSED!" CRESET << endl;
        paused = true;

	} else{
        if(paused){
            startSubscribers();
		    cout << HEADER_PRINT GREEN "STARTED!" CRESET << endl;
        }
        paused = false;
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
    nh_private.param("topic_measure_type" , topic_measure_type_str, string("Float64") );
    
    nh_private.param("topic_force_command" , topic_force_command_str, string("command_force") );
	string topic_goal_speed("");
    nh_private.param("topic_goal_speed" , topic_goal_speed, string("goal_speed") );

    string pause_service("");
    nh_private.param("pause_service" , pause_service, string("pause") );
    nh_private.param("start_in_pause" , paused, false );

    nh_private.param("control_gain" , control_gain, (float)1.0 );
    nh_private.param("max_force" , max_force, (float)20.0 );

	 // Publisher
     if(!paused){
        startSubscribers();
     }
	 velPub = nh_public->advertise<std_msgs::Float32>( topic_goal_speed,1);

    ros::ServiceServer servicePause = nh_public->advertiseService(pause_service, pause_callbk);

    control_gain = fabs(control_gain);
    max_force = fabs(max_force);

    signal(SIGINT, intHandler);

    ros::spin();

	sendZeroVel();

    return 0;
}

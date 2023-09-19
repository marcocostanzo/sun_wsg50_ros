#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sun_ros_msgs/Float64Stamped.h>

#define CIL_H 0.15
#define CIL_D 0.02
#define MAX_GRASP_FORCE 12.0
#define YELLOW_GRASP_FORCE (MAX_GRASP_FORCE/2.0)

double red_color = 0.0;
double green_color = 1.0;
double grasp_force_n = 0.0;
void grasp_force_cb( const sun_ros_msgs::Float64Stamped::ConstPtr& msg )
{

  double grasp_force = fabs(msg->data);
  if(grasp_force < YELLOW_GRASP_FORCE){
    red_color = grasp_force/YELLOW_GRASP_FORCE;
    green_color = 1.0;
  } else if( grasp_force < MAX_GRASP_FORCE ) {
    red_color = 1.0;
    green_color = 1.0 - ( grasp_force - YELLOW_GRASP_FORCE ) / ( MAX_GRASP_FORCE - YELLOW_GRASP_FORCE );
  } else {
    red_color = 1.0;
    green_color = 0.0;
  }

  grasp_force_n = grasp_force / MAX_GRASP_FORCE;

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "grasp_force_visualizer");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");

  std::string frame_id;
  nh_private.param( "frame_id", frame_id, std::string("base_link") );
  double hz;
  nh_private.param( "rate", hz, 3.0 );
  double scale;
  nh_private.param( "scale", scale, 1.0 );
  std::string grasp_force_topic_str;
  nh_private.param( "grasp_force_topic", grasp_force_topic_str, std::string("grasp_force") );

  double put_marker_down;
  nh_private.param( "put_marker_down", put_marker_down, 1.0 );

  ros::Subscriber sub_grasp_force = n.subscribe(grasp_force_topic_str, 1, grasp_force_cb);
  

  ros::Rate r(hz);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("grasp_force_marker", 1);

  // Set our shape type to be a cylinder
  uint32_t shape = visualization_msgs::Marker::CYLINDER;

  while (ros::ok())
  {

    ros::spinOnce();

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "wsg50";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.0;//-(CIL_H*(1.0-grasp_force_n) )/2.0*scale;
    marker.pose.position.y = put_marker_down*( 0.025 + (CIL_D/2.0) )*scale; //up
    marker.pose.position.z = (0.07 - (CIL_D/2.0) )*scale; //to tcp
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.7071068;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.7071068;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = CIL_D*scale;
    marker.scale.y = CIL_D*scale;
    marker.scale.z = (CIL_H*grasp_force_n)*scale;
    if(marker.scale.z == 0.0) marker.scale.z = 0.001;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = red_color;
    marker.color.g = green_color;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      //ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    r.sleep();
  }
}
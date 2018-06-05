#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

double odom_x=0.0, odom_y=0.0;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  odom_x = odom_msg->pose.pose.position.x;
  odom_y = odom_msg->pose.pose.position.y;
  //ROS_INFO("Odometry: (x, y) = (%2.1f, %2.1f)", odom_x, odom_y);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Subscriber odom_sub;
  odom_sub = n.subscribe("odom", 1000, odom_callback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 4.0;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // Initially show the marker at the pickup zone
    ROS_INFO("Publish the marker at the pickup zone.");
    while (marker_pub.getNumSubscribers() < 1){
      if (!ros::ok()){
	return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    ros::Duration(1.0).sleep();

    double marker_dist = 100.0;
    const double dist_threshold = 0.3;
    while( marker_dist > dist_threshold ){
      marker_dist = sqrt( pow(odom_x-marker.pose.position.x, 2) + pow(odom_y-marker.pose.position.y, 2) );
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      ROS_INFO("marker_dist: %2.1f, threshold: %2.1f", marker_dist, dist_threshold);
    }

    // Hide the marker once your robot reaches the pickup zone
    ROS_INFO("Reached a marker!");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // Wait 5 seconds to simulate a pickup
    ROS_INFO("Pause 5 seconds.");
    ros::Duration(5.0).sleep();

    // Show the marker at the drop off zone once your robot reaches it
    marker.pose.position.x = 4.0;
    marker.pose.position.y = -0.1;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    double drop_dist = 100.0;
    while( drop_dist > dist_threshold ){
      drop_dist = sqrt( pow(odom_x-marker.pose.position.x, 2) + pow(odom_y-marker.pose.position.y, 2) );
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      ROS_INFO("drop_dist: %2.1f, threshold: %2.1f", drop_dist, dist_threshold);
    }

    // Set the marker action again.
    ROS_INFO("Publish the marker at the drop off zone.");
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
    
    ROS_INFO("Pause 10 seconds.");
    ros::Duration(10.0).sleep();


    r.sleep();
  }
}

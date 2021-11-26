#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <marvelmind_nav/hedge_pos.h>

  ros::Subscriber odom_sub;
  ros::Publisher odom_pub;

 void Marker_Callback(const visualization_msgs::Marker::ConstPtr& msg)
 {
     ROS_INFO("I heard Marker");
     ros::Time current_time = ros::Time::now();
     tf::TransformBroadcaster odom_broadcaster;

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "hedge_link";

    odom_trans.transform.translation.x = msg->pose.position.x;
    odom_trans.transform.translation.y = msg->pose.position.y;
    odom_trans.transform.translation.z = msg->pose.position.z;
    odom_trans.transform.rotation = msg->pose.orientation;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x= msg->pose.position.x;
    odom.pose.pose.position.y= msg->pose.position.y;
    odom.pose.pose.position.z= msg->pose.position.z;
    odom.pose.pose.orientation = msg->pose.orientation;
 
    //publish the message
    odom_pub.publish(odom);
 }
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hedge_odom");
  ros::NodeHandle n;
 

  odom_pub = n.advertise<nav_msgs::Odometry>("hedge_odom", 50);
  odom_sub = n.subscribe("/visualization_marker", 1000, Marker_Callback);

 
    ros::spin();     
    return 0;

}
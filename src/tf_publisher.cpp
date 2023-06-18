#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class tf_publisher{
  
  public:
    tf_publisher(){
      sub = n.subscribe("/t265/odom", 1000, &tf_publisher::callback, this);
    }


  void callback(const nav_msgs::Odometry& msg){
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);    
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "t265"));
  }


  private:
    ros::NodeHandle n; 
    tf::TransformBroadcaster br; // tf is still a topic
    ros::Subscriber sub;
};


int main(int argc, char **argv){
 ros::init(argc, argv, "tf_publisher");
 tf_publisher tf_publisher;
 ros::spin();
 return 0;
}


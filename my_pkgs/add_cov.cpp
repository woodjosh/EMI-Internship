#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>




/*
 * Subscribes to the pose outputs from orbslam and adds a covariance so messages can be used by 
 * the robot_localization package. If orbslam is lost and is no longer updating position, it
 * publishes message at the last known location so that robot_localization does not drift. 
 */
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb_slam2_mono/pose_cov", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/orb_slam2_mono/pose", 1, &SubscribeAndPublish::callback, this);
  }

  double get_publish_time()
  {
    return last_publish_time.toSec(); 
  }  

  void callback(const geometry_msgs::PoseStamped& input)
  {
    last_publish_time = ros::Time::now(); 
    geometry_msgs::PoseWithCovarianceStamped output;
    output.header = input.header;
    output.pose.pose = input.pose;
    output.pose.covariance[0] = 0; //x  
    output.pose.covariance[7] = 0; //y
    output.pose.covariance[14] = 0; //z
    output.pose.covariance[21] = 0; //rot x
    output.pose.covariance[28] = 0; //rot y
    output.pose.covariance[35] = 0; //rot z
    
    last_msg = output; 
    pub_.publish(output);
  }

  void publish_dummy_msg()
  {
    last_publish_time = ros::Time::now(); 
    geometry_msgs::PoseWithCovarianceStamped output;
    output.header.stamp = ros::Time::now(); 
    output.header.frame_id = "map"; 
    output.pose.pose = last_msg.pose.pose; 
    output.pose.covariance[0] = 0; //x  
    output.pose.covariance[7] = 0; //y
    output.pose.covariance[14] = 0; //z
    output.pose.covariance[21] = 0; //rot x
    output.pose.covariance[28] = 0; //rot y
    output.pose.covariance[35] = 0; //rot z

    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  
  ros::Time last_publish_time; 

  geometry_msgs::PoseWithCovarianceStamped last_msg;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "add_cov_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  double rate = 30; //hz
  ros::Rate r(rate);
  while(ros::ok()) 
  {
    ros::spinOnce(); 
    double curr_time = ros::Time::now().toSec(); 
    double last_publish_time = SAPObject.get_publish_time(); 
    if(curr_time-last_publish_time > 1.0/rate*2.0)
    { 
      SAPObject.publish_dummy_msg();  
    }
    r.sleep();  
  }

  return 0;
}

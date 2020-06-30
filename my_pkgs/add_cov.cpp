#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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

  void callback(const geometry_msgs::PoseStamped& input)
  {
    geometry_msgs::PoseWithCovarianceStamped output;
    output.header = input.header;
    output.pose.pose = input.pose;
    output.pose.covariance[0] = 0.1; //x  
    output.pose.covariance[7] = 0.1; //y
    output.pose.covariance[14] = 0.1; //z
    output.pose.covariance[21] = 1e-8; //rot x
    output.pose.covariance[28] = 1e-8; //rot y
    output.pose.covariance[35] = 1e-8; //rot z
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "add_cov_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}

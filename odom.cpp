#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <bits/stdc++.h>


using namespace std;

using namespace std::chrono_literals;
using std::placeholders::_1;


class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

    wheel_sub_ = this->create_subscription<std_msgs::msg::String>("robot_wheel_vel", 10, std::bind(&Odometry::topic_callback, this, _1));


      startTime_ = this->get_clock()->now();
      endTime_ = this->get_clock()->now();
      posX_ = posY_ = 0;
      posTH_ = 0;

     tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
     tf_static_broadcaster2_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      make_transforms();
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
      vector <string> tokens;
      stringstream check1(msg->data.c_str());
      string intermediate;
      while(getline(check1, intermediate, ','))  {
        tokens.push_back(intermediate);
    }

      if(tokens.size() != 2)
          return;

      left_vel_ = std::stof(tokens[0]);
      right_vel_ = std::stof(tokens[1]);

      send_odom();
  }

  void send_odom() {
    double dxy,dth;
    double dt;
    double DL,DR;


    startTime_ = this->get_clock()->now();

    rclcpp::Duration interval(startTime_ - endTime_);

    dt = interval.seconds();

    DL = dt * left_vel_;
    DR = dt * right_vel_;


      dxy = (DL + DR) / 2;
      dth = (DR - DL) / 0.24;     //0.24 = robot width

       posX_ += dxy * cos(posTH_);
       posY_ += dxy * sin(posTH_);
       posTH_ += dth;


    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = posX_;
    t.transform.translation.y = posY_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;

    q.setRPY(0, 0, posTH_);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);


       tf2::Quaternion odom_quat;
       odom_quat.setRPY( 0, 0, posTH_);


       nav_msgs::msg::Odometry odom;
       odom.header.stamp = this->get_clock()->now();;
       odom.header.frame_id = "odom";

       odom.pose.pose.position.x = posX_;
       odom.pose.pose.position.y = posY_;
       odom.pose.pose.position.z = 0.0;

       odom.pose.pose.orientation = tf2::toMsg(odom_quat);;

       odom.child_frame_id = "base_footprint";
       odom.twist.twist.linear.x = dxy / dt;
       odom.twist.twist.linear.y = 0;
       odom.twist.twist.angular.z = dth / dt;

       //publish the message
       odom_pub_->publish(odom);

       endTime_ = this->get_clock()->now();
  }


 void make_transforms()  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_footprint";
    t.child_frame_id = "base_link";

    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;


    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 1;

    tf_static_broadcaster_->sendTransform(t);

   geometry_msgs::msg::TransformStamped t2;

    t2.header.stamp = this->get_clock()->now();
    t2.header.frame_id = "base_link";
    t2.child_frame_id = "laser";

    t2.transform.translation.x = 0;
    t2.transform.translation.y = 0;
    t2.transform.translation.z = 0;

    t2.transform.rotation.x = 0;
    t2.transform.rotation.y = 0;
    t2.transform.rotation.z = 0;
    t2.transform.rotation.w = 1;


    tf_static_broadcaster2_->sendTransform(t2);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wheel_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster2_;


  float left_vel_;
  float right_vel_;

  rclcpp::Time startTime_;
  rclcpp::Time endTime_;
  double posX_;
  double posY_;
  double posTH_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}

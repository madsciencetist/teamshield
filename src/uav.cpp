
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>

Eigen::Vector3d current_location, goal;
ros::Publisher current_location_pub;

void goalCallback(geometry_msgs::Point new_goal)
{
  goal[0] = new_goal.x;
  goal[1] = new_goal.y;
  goal[2] = new_goal.z;
}

void timerCallback(const ros::TimerEvent& e)
{
  double dt = (e.current_real - e.last_real).toSec();

  Eigen::Vector3d travel_vector = goal - current_location;
  if(travel_vector.norm() > 1) {
    travel_vector.normalize();
  }

  double max_speed = 5.0; // meters per second
  double climb_rate = 0.5; // meters per second
  double descent_rate = 1.5; // meters per second

  Eigen::Vector3d velocity = travel_vector * max_speed;

  if(velocity[2] > climb_rate)
    velocity[2] = climb_rate;
  else if(velocity[2] < -descent_rate)
    velocity[2] = -descent_rate;

  current_location += velocity * dt;

  geometry_msgs::Point current_location_msg;
  current_location_msg.x = current_location[0];
  current_location_msg.y = current_location[1];
  current_location_msg.z = current_location[2];
  current_location_pub.publish(current_location_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav");
  ros::NodeHandle nh;

  current_location.setZero();
  goal.setZero();

  ros::Subscriber goal_sub = nh.subscribe("goal", 1, goalCallback);
  current_location_pub = nh.advertise<geometry_msgs::Point>("current_location", 0);
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

  ros::spin();

  return 0;
}


#include <ros/ros.h>
#include <ros/console.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>

class Planner
{

  public:
    Planner():
      tfListener(tfBuffer)
  {
    ros::NodeHandle nhp("~");

    goal_pose_pub = nhp.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);
    pose_sub = nhp.subscribe<geometry_msgs::PoseStamped>("/pose", 1, &poseCb, this);
    pose_sub = nhp.subscribe<geometry_msgs::PoseStamped>("/quadtree", 1, &quadtreeCb, this);
    getNeighborsClient = nhp.serviceClient<teamShield::GetNeighbors>("get_neighbors");

    map_width = nhp.param("map_width" , 100);
    map_height = nhp.param("map_height" , 100);


    //Sensor configuration settings - Support sonar or lidar inputs ************

    //top_sonar_present = nhp.param("top_sonar_present", true);
    iteration = 0;

  }

  private:

    void poseCb(const geometry_msgs::PoseStamped &pose);
    void quadtreeCb(const geometry_msgs::PoseStamped &qt_temp);
    ros::ServiceClient getNeighborsClient;

    geometry_msgs::PoseStamped move_forward(geometry_msgs::PoseStamped current_pose, int level);
    geometry_msgs::PoseStamped move_right(geometry_msgs::PoseStamped current_pose, int level);
    geometry_msgs::PoseStamped move_left(geometry_msgs::PoseStamped current_pose, int level);

    geometry_msgs::Vector3 vel_correction;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::NodeHandle nh;

    ros::Publisher goal_pose_pub;
    ros::Subscriber pose_sub;
    int iteration;
    double map_width, map_height;
    double neighbors[8];

};

void Planner::poseCb(const geometry_msgs::PoseStamped &pose){

  //assume the uav's level is storred in its pose.z //TODO rounding, non-zero
  int level = pose.pose.position.z;

  //Note: the resolution of the camera fov is defined by the uav level and map_height
  //level 1 is responsible for clearning unknows space. always move forward
  if (level == 1){
    //send goal to end of lane...
    //move_forward(pose, level);
  }
  else {
    // if neighrbors less than cuttoff, move forward
    teamShield::GetNeighbors srv;
    if (getNeighbors){

    }


    // else go to neighbor with the highest prob
      // if highest prob has duplicates, bias forward

  }


}
void Planner::quadtreeCb(const geometry_msgs::PoseStamped &qt_temp)
{
  qt = *qt_temp;
  //do stuff
}

geometry_msgs::PoseStamped Planner::move_forward(geometry_msgs::PoseStamped current_pose,int level){
  //how much to move forward?
  //assume uav's move along the height direction

  double dy = (map_height/std::pow(2,level));
  geometry_msgs::PoseStamped new_pose;
  new_pose.header.stamp = ros::Time::now();
  new_pose.pose.position.x = current_pose.pose.position.x;
  new_pose.pose.position.y = current_pose.pose.position.y + dy;
  new_pose.pose.position.z = current_pose.pose.position.z;

  return new_pose;
}
geometry_msgs::PoseStamped Planner::move_right(geometry_msgs::PoseStamped current_pose, int level){

  double dx = (map_height/std::pow(2,level));
  geometry_msgs::PoseStamped new_pose;
  new_pose.header.stamp = ros::Time::now();
  new_pose.pose.position.x = current_pose.pose.position.x + dx;
  new_pose.pose.position.y = current_pose.pose.position.y;
  new_pose.pose.position.z = current_pose.pose.position.z;

  return new_pose;
}
geometry_msgs::PoseStamped Planner::move_left(geometry_msgs::PoseStamped current_pose, int level){

  double dx = -(map_height/std::pow(2,level));
  geometry_msgs::PoseStamped new_pose;
  new_pose.header.stamp = ros::Time::now();
  new_pose.pose.position.x = current_pose.pose.position.x + dx;
  new_pose.pose.position.y = current_pose.pose.position.y;
  new_pose.pose.position.z = current_pose.pose.position.z;

  return new_pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner");
  Planner planner;
  ros::spin();
  return 0;
}

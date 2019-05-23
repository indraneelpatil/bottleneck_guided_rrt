#include <ros/ros.h>
#include "std_msgs/String.h"

#include <sstream>


#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>

ros::Publisher pub;
/*

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryActionGoal > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryActionGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  
  control_msgs::FollowJointTrajectoryActionGoal armExtensionTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryActionGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.goal.trajectory.joint_names.push_back("joint_1");
    goal.goal.trajectory.joint_names.push_back("joint_2");
    goal.goal.trajectory.joint_names.push_back("joint_3");
    goal.goal.trajectory.joint_names.push_back("joint_4");
    goal.goal.trajectory.joint_names.push_back("joint_5");
    goal.goal.trajectory.joint_names.push_back("joint_6");
    goal.goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.goal.trajectory.points[ind].positions.resize(7);
    goal.goal.trajectory.points[ind].positions[0] = 0.0;
    goal.goal.trajectory.points[ind].positions[1] = 0.0;
    goal.goal.trajectory.points[ind].positions[2] = 0.0;
    goal.goal.trajectory.points[ind].positions[3] = 0.0;
    goal.goal.trajectory.points[ind].positions[4] = 0.0;
    goal.goal.trajectory.points[ind].positions[5] = 0.0;
    goal.goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    goal.goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.goal.trajectory.points[ind].positions.resize(7);
    goal.goal.trajectory.points[ind].positions[0] = -0.3;
    goal.goal.trajectory.points[ind].positions[1] = 0.2;
    goal.goal.trajectory.points[ind].positions[2] = -0.1;
    goal.goal.trajectory.points[ind].positions[3] = -1.2;
    goal.goal.trajectory.points[ind].positions[4] = 1.5;
    goal.goal.trajectory.points[ind].positions[5] = -0.3;
    goal.goal.trajectory.points[ind].positions[6] = 0.5;
    // Velocities
    goal.goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};
*/

void Callback(const moveit_msgs::ExecuteTrajectoryActionGoal::ConstPtr& msg)
{
  ROS_INFO("Moveit trajectory received ");

	control_msgs::FollowJointTrajectoryActionGoal goal;

	goal.header=msg->header;
	goal.goal_id=msg->goal_id;
	goal.goal.trajectory=msg->goal.trajectory.joint_trajectory;

	/*
  	// When to start the trajectory: 1s from now
   // goal.goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

    // First, the joint names, which apply to all waypoints
    goal.goal.trajectory.joint_names.push_back("joint_1");
    goal.goal.trajectory.joint_names.push_back("joint_2");
    goal.goal.trajectory.joint_names.push_back("joint_3");
    goal.goal.trajectory.joint_names.push_back("joint_4");
    goal.goal.trajectory.joint_names.push_back("joint_5");
    goal.goal.trajectory.joint_names.push_back("joint_6");

    // We will have two waypoints in this goal trajectory
    goal.goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.goal.trajectory.points[ind].positions.resize(6);
    goal.goal.trajectory.points[ind].positions[0] = 0.0;
    goal.goal.trajectory.points[ind].positions[1] = 0.0;
    goal.goal.trajectory.points[ind].positions[2] = 0.0;
    goal.goal.trajectory.points[ind].positions[3] = 0.0;
    goal.goal.trajectory.points[ind].positions[4] = 0.0;
    goal.goal.trajectory.points[ind].positions[5] = 0.0;
    // Velocities
    goal.goal.trajectory.points[ind].velocities.resize(6);
    for (size_t j = 0; j < 6; ++j)
    {
      goal.goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.goal.trajectory.points[ind].positions.resize(6);
    goal.goal.trajectory.points[ind].positions[0] = -0.3;
    goal.goal.trajectory.points[ind].positions[1] = 0.2;
    goal.goal.trajectory.points[ind].positions[2] = -0.1;
    goal.goal.trajectory.points[ind].positions[3] = -1.2;
    goal.goal.trajectory.points[ind].positions[4] = 1.5;
    goal.goal.trajectory.points[ind].positions[5] = -0.3;
    // Velocities
    goal.goal.trajectory.points[ind].velocities.resize(6);
    for (size_t j = 0; j < 6; ++j)
    {
      goal.goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);
	*/
   std::cout<<"Publishing trajectory"<<std::endl;
   usleep(100000);
   pub.publish(goal);
  usleep(100000);


}


int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/execute_trajectory/goal", 1000, Callback);
  pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/joint_trajectory_action/goal", 1000);
	//while (ros::ok())
  
  	
    ros::spin();


/*
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
  */
  return 0;
}
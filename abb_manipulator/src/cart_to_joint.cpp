#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()
#include <iostream>
#include <cstdlib>


#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

float SIGN(float x);
float NORM(float a, float b, float c, float d);
bool ik_fail=false;

int sequence=1;
ros::Publisher pub;
std::vector<float> previous_angles;

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IK_VERSION 61
#include "ik_final.cpp"

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal // for IKFast 54
#endif


bool check_jointlimits(const std::vector<float> angles)
{
	bool flag=false;
	for(int g=0;g<angles.size();g++)
		{	

			switch(g)
			{

				case 0:
					{
					if((angles[g]>5.933)&&(angles[g]<-5.933))
						flag=true;
					break;
					}
				case 1:
					{
					if((angles[g]>2.443)&&(angles[g]<-2.443))
						flag=true;
					break;
					}
				case 2:
					{
					if((angles[g]>2.3557)&&(angles[g]<-2.3557))
						flag=true;
					break;
					}
				case 3:
					{
					if((angles[g]>5.235)&&(angles[g]<-5.235))
						flag=true;
					break;
					}
				case 4:
					{
					if((angles[g]>4.0135)&&(angles[g]<-4.0135))
						flag=true;
					break;
					}
				case 5:
					{
					if((angles[g]>6.282)&&(angles[g]<-6.282))
						flag=true;
					break;
					}
			}

		}
	
	if(flag==true)
	{
		std::cout<<"Joint Limits are violated!!!!!!!!!"<<std::endl;
	}
 	return flag;

}


void Callback2(const sensor_msgs::JointState::ConstPtr& msg)
{	
	//ROS_INFO("Robot Start configuration received");
	previous_angles.clear();
	for(int i=0;i<msg->position.size();i++)
	{	
		previous_angles.push_back(msg->position[i]);

	}

	check_jointlimits(previous_angles);

}

void Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
		ROS_INFO("Cartesian Space to Joint Space trajectory conversion started");
  //ROS_INFO("I heard: [%s]", msg->data.c_str());

	ros::Time begin = ros::Time::now();
    if(previous_angles.size()>2)
    {
		control_msgs::FollowJointTrajectoryActionGoal goal;
		goal.header=msg->header;
		goal.header.seq=sequence;
		sequence++;
		goal.goal_id.stamp=goal.header.stamp;
		std::string text = "Planner ";
		text += std::to_string(sequence);
		goal.goal_id.id=text;
		// First, the joint names, which apply to all waypoints
   		 goal.goal.trajectory.joint_names.push_back("joint_1");
   		 goal.goal.trajectory.joint_names.push_back("joint_2");
   		 goal.goal.trajectory.joint_names.push_back("joint_3");
    	goal.goal.trajectory.joint_names.push_back("joint_4");
    	goal.goal.trajectory.joint_names.push_back("joint_5");
    	goal.goal.trajectory.joint_names.push_back("joint_6");
   		 goal.goal.trajectory.header.frame_id="world";



//Iterating through all the waypoints
    for(int it=0;it<msg->poses.size();it++)
    {
    	float x=msg->poses[it].position.x;
    	float y=msg->poses[it].position.y;
    	float z=msg->poses[it].position.z;
    	float or1= 0  ;//msg->poses[it].orientation.x;
    	float or2= 0  ; //msg->poses[it].orientation.y;
    	float or3= 1  ; //msg->poses[it].orientation.z;
    	float or4= 0 ; //msg->poses[it].orientation.w;

    	std::cout<<"Finding Inverse of the "<<it<<" waypoint "<<std::endl;
    	IKREAL_TYPE eerot[9],eetrans[3];
    	unsigned int num_of_joints = GetNumJoints();
    	unsigned int num_free_parameters = GetNumFreeParameters();

     	IkSolutionList<IKREAL_TYPE> solutions;
     	std::vector<IKREAL_TYPE> vfree(num_free_parameters);

     	    eetrans[0] = x;
            eetrans[1] = y;
            eetrans[2] = z;

            // Convert input effector pose, in w x y z quaternion notation, to rotation matrix. 
            // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
            double qw = or1;
            double qx = or2;
            double qy = or3;
            double qz = or4;
            const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
            qw *= n;
            qx *= n;
            qy *= n;
            qz *= n;
            eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
            eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
            eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

            //for(std::size_t i = 0; i < vfree.size(); ++i)
             //   vfree[i] = atof(argv[13+i]);

            // for IKFast 56,61
            bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
            if( !bSuccess ) {
                fprintf(stderr,"Failed to get ik solution\n");
               // return -1;
                ik_fail=true;
            }

            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

            printf("Found %d ik solutions:\n", num_of_solutions ); 
            std::vector<IKREAL_TYPE> solvalues(num_of_joints);
             
             std::vector<float> best_solution;
             float lowest_difference=50;
             
             for(std::size_t i = 0; i < num_of_solutions; ++i) 
             {		
             		float solution_difference=0;
             		// for IKFast 56,61
                	const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                	int this_sol_free_params = (int)sol.GetFree().size(); 

                	printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
                	std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
                	// for IKFast 56,61
                	sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
                	
                	for( std::size_t j = 0; j < solvalues.size(); ++j)
                	{
                    	printf("%.15f, ", solvalues[j]);
                    	solution_difference=solution_difference+abs(solvalues[j]-previous_angles[j]);

                	}
                	std::cout<<"Solution difference of "<<i<<"solution is: "<<solution_difference<<std::endl;
  // Trying to find the closest solution to previous configuration
                	if(solution_difference<lowest_difference)
                	{	
                		std::cout<<"Since"<<solution_difference<<"is lower than "<<lowest_difference<<" Better solution found!! "<<std::endl;
                		lowest_difference=solution_difference;
                		best_solution.clear();

                			for(int y=0;y<solvalues.size();++y)
                			{
                				best_solution.push_back(solvalues[y]);
                			}

                	}

                	printf("\n");
    			}

    			std::cout<<"lowest difference was :"<<lowest_difference<<std::endl;

    			if(!ik_fail)
    			{
    					check_jointlimits(best_solution);
   //add best solution to trajectory
    					previous_angles=best_solution;
    					trajectory_msgs::JointTrajectoryPoint point;
    					for(int j=0;j<best_solution.size();j++)
    					{
    				
    						point.positions.push_back(best_solution[j]);

    					}
    			
    					ros::Time now = ros::Time::now();
    					point.time_from_start=now-begin;
    					goal.goal.trajectory.points.push_back(point);

    				}
    			else
    					ik_fail=false;
    			
    			std::cout<<"******************************"<<std::endl;
    			std::cout<<"  "<<std::endl;

}
			std::cout<<"Publishing Joint Space trajectory"<<std::endl;
   			usleep(100000);
  			 pub.publish(goal);
  			usleep(100000);
  		}
  		else
  			ROS_INFO("Waiting for Robot Start Configuration");

}


//Cartesian trajectory to Joint Space trajectory converter
int main(int argc, char **argv)
{
	ros::init(argc, argv, "cart_joint_conv");
	ros::NodeHandle n;
	ros::Subscriber sub2=n.subscribe("joint_states",1000,Callback2);
	ros::Subscriber sub = n.subscribe("final_rrt_trajectory", 1000, Callback);
	pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/joint_trajectory_action/goal", 1000);
	ros::spin();
	 return 0;
}
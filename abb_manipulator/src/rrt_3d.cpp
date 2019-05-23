#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mtrand.h"
#include "H5Easy.h"
#include <sstream>
#include <tuple>
#include <vector>
#include <iostream>
#include <math.h>
#include <chrono>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/ccd/motion.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"


#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <iostream>
#include <octomap_msgs/conversions.h>
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <stdlib.h>
#include <boost/foreach.hpp>

#include <Eigen/Geometry> 
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf_conversions/tf_eigen.h"


using namespace fcl;
int a,prev_k=0;

LoadH5 data;
visualization_msgs::MarkerArray points,line,points_smooth;
visualization_msgs::Marker points_temp,line_temp,line_trajectory,simplified_trajectory,pnt_temp;;
float xdim,ydim,resolution,Xstartx,Xstarty,Xstartz,init_map_x,init_map_y,init_map_z,map_width,map_height,map_length,goal_x,goal_y,goal_z;
float xr,yr,zr,random_indice;
int i,j=0;
geometry_msgs::Point p;  
bool found=true;
bool goal_search=true;
bool waiting_command=true;
bool octomap_received=false;
bool collision_result=true;
bool collision_flag=false;
bool collision_flag2=false;

bool s,g=false;
int parent_index;
std::vector<float> goal_command,start_command;
std::deque<int> index_path;

 // Random Point marker shape
  uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t shape2 = visualization_msgs::Marker::SPHERE;

  //Max travel allowance for each branch/ step size                               //PARAMETER 1
  float eta = 0.15;                                                               //PARAMETER 2 Max Iterations

  std::vector<geometry_msgs::Pose> plan,simplify_plan,smooth_plan,pruned_path,smoothened_path;
  
  geometry_msgs::PoseArray trajectory_final;
  ros::Publisher pub3,pub5,pub6;
  ros::Publisher visPub ;

    // create empty tree with resolution 0.1
    octomap::OcTree* octTree = new octomap::OcTree(0.1);
    std::shared_ptr<fcl::CollisionGeometry> tree_obj;
    
    Eigen::Quaternionf q;
    tf::Quaternion tf_q;
    int pruned_size=0;

//sign function
float sign(float n)
{
      if (n<0.0){
          return -1.0;}
      
      else{return 1.0;}
}


//Norm function 
float Norm(std::vector<float> x1,std::vector<float> x2)
{
          return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2)+pow((x2[2]-x1[2]),2))	,0.5);

}


//Nearest function - return nearest point as well as its index
std::tuple<std::vector<float>,int> Nearest(  std::vector< std::vector<float>  > V, std::vector<float>  x)

  {      float min=Norm(V[0],x);
        int min_index;
        float temp;

        for (int j=0;j<V.size();j++)
        {
            temp=Norm(V[j],x);
            if (temp<=min){
            min=temp;
            min_index=j;}

          }
        return  std::make_tuple(V[min_index],min_index);
        //return V[min_index];
}

std::vector<geometry_msgs::Pose> BuildPlan(std::vector< std::vector<float>> V)
{
        ROS_INFO("Building the plan.");
        int number_of_waypoints=0;
        
        int current_index=(V.size()-1);
        while(current_index>0)
        {
          std::cout<<current_index<<std::endl;
          index_path.push_front(current_index);
          current_index=V[current_index][3];
          number_of_waypoints=number_of_waypoints+1;
        }
        index_path.push_front(0);
        number_of_waypoints=number_of_waypoints+1;
        std::cout<<"Number of waypoints in the trajectory: "<<number_of_waypoints<<std::endl;
        // build the plan back up in PoseStamped messages

        for(int k : index_path)
        {
          //std::cout<<k<<std::endl;
          geometry_msgs::Pose pos;
          pos.position.x = V[k][0];
          pos.position.y = V[k][1];
          pos.position.z = V[k][2];

          pos.orientation = tf::createQuaternionMsgFromYaw(0);
          plan.push_back(pos);
          //trajectory_final.poses.push_back(pos);
           
           // Visualizing the Final Trajectory from Start to Goal 
           if(a==0)
           {
           p.x=V[0][0];       // Add the start to the trajectory
           p.y=V[0][1]; 
           p.z=V[0][2];
           line_trajectory.points.push_back(p);
           p.x=V[k][0]; 
           p.y=V[k][1]; 
           p.z=V[k][2];
           line_trajectory.points.push_back(p);
           pub3.publish(line_trajectory); 
           a=a+1;
          }
          else
          {
           p.x=V[prev_k][0]; 
           p.y=V[prev_k][1]; 
           p.z=V[prev_k][2];
           line_trajectory.points.push_back(p);
           p.x=V[k][0]; 
           p.y=V[k][1]; 
           p.z=V[k][2]; 
           line_trajectory.points.push_back(p);
           pub3.publish(line_trajectory); 
          }
          prev_k=k;
        }


        return plan;

}


void Callback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->marker_name.c_str());
  std::string str1("1");
  std::string str2("2");
  
  //std::cout<<"Message received"<<std::endl; 
  if((int(msg->event_type)==5)&&(msg->marker_name.c_str()==str1))
  {
    std::cout<<"Start point received :"<<msg->pose.position.x<< " "<<msg->pose.position.y<< " "<<msg->pose.position.z<< " "<<std::endl;
    start_command.push_back(msg->pose.position.x);
    start_command.push_back(msg->pose.position.y);
    start_command.push_back(msg->pose.position.z);
    s=true;

  }

    else if((int(msg->event_type)==5)&&(msg->marker_name.c_str()==str2))
  {
    std::cout<<"End Goal received :"<<msg->pose.position.x<< " "<<msg->pose.position.y<< " "<<msg->pose.position.z<< " "<<std::endl;
    goal_command.push_back(msg->pose.position.x);
    goal_command.push_back(msg->pose.position.y);
    goal_command.push_back(msg->pose.position.z);
    g=true;
  }

}


void octomap_Callback(const octomap_msgs::Octomap::ConstPtr& msg)                                   //PARAMETER 3
{

        octomap_received=true;

        //octomap.header.stamp = ros::Time::now();
        octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
        octTree = dynamic_cast<octomap::OcTree*>(tree);
        std::cout<<"Octomap Received"<<std::endl;



}


void generateBoxesFromOctomap(std::vector<CollisionObject*>& boxes, OcTree& tree)
    {

        std::vector<std::array<FCL_REAL, 6> > boxes_ = tree.toBoxes();
        for(std::size_t i = 0; i < boxes_.size(); ++i)
        {
            FCL_REAL x = boxes_[i][0];
            FCL_REAL y = boxes_[i][1];
            FCL_REAL z = boxes_[i][2];
            FCL_REAL size = boxes_[i][3];
            FCL_REAL cost = boxes_[i][4];
            FCL_REAL threshold = boxes_[i][5];
            Box* box = new Box(size, size, size);
            box->cost_density = cost;
            box->threshold_occupied = threshold;
            CollisionObject* obj = new CollisionObject(std::shared_ptr<CollisionGeometry>(box), Transform3f(Vec3f(x, y, z)));
            boxes.push_back(obj);
        }
    }



bool isStateValid(std::vector<float> point)
{
     // convert the octomap::octree to fcl::octree fcl_octree object
     //std::cout<<"Collision checking started"<<std::endl;
      fcl::OcTree* tree2 = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(octTree));
      //tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree2);
      std::vector<fcl::CollisionObject*> boxes;
      generateBoxesFromOctomap(boxes, *tree2);
      // new fcl::OcTree(0.5);
      Transform3f tf0;
      tf0.setIdentity();
      tf0.setTranslation(Vec3f(point[0], point[1],point[2]));
      std::shared_ptr<Sphere> Shpere0(new Sphere(0.1));
      CollisionObject co0(Shpere0, tf0);
      for(size_t i = 0; i < boxes.size(); ++i)
        {
            CollisionObject* box =  boxes[i];
            static const int num_max_contacts = std::numeric_limits<int>::max();
            static const bool enable_contact = true ;
            fcl::CollisionResult result;
            fcl::CollisionRequest request(num_max_contacts, enable_contact);
            fcl::collide(&co0, box, request, result);

            if (result.isCollision() == true )
            {
                
                //collisionFlag.data = true ;
                //collisionFlagPub.publish(collisionFlag);
                //std::cout<<"Collision Detected"<<std::endl;
                collision_flag=true;
                break;
            }
            else
            {
                //std::cout<<"State is safe"<<std::endl;
                collision_flag=false;
            }
        }
  
      return collision_flag;

}

//used for visualizing the cylindrical collision object for line checking
void drawSphere(Vec3f vec )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vec[0];
        marker.pose.position.y = vec[1];
        marker.pose.position.z = vec[2];
        //std::cout<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
        marker.pose.orientation.x = float(q.x());
        marker.pose.orientation.y = float(q.y());
        marker.pose.orientation.z = float(q.z());
        marker.pose.orientation.w = float(q.w());
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration();
        visPub.publish( marker );
    }




bool isLineValid(std::vector<float> point1,std::vector<float> point2)
{
       std::cout<<"Line Collision Checking Started"<<std::endl;
        // convert the octomap::octree to fcl::octree fcl_octree object
     //std::cout<<"Collision checking started"<<std::endl;
      fcl::OcTree* tree2 = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(octTree));
      //tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree2);
      std::vector<fcl::CollisionObject*> boxes;
      generateBoxesFromOctomap(boxes, *tree2);
      Transform3f tf1;
      tf1.setIdentity();
      tf1.setTranslation(Vec3f((point1[0]+point2[0])/2, (point1[1]+point2[1])/2,(point1[2]+point2[2])/2));
      
      //find 3D rotation matrix of line
      float magnitude =pow(pow(point2[2] - point1[2],2)+pow(point2[1] - point1[1],2)+pow(point2[0] - point1[0],2),0.5);
        
          //std::cout<<magnitude<<std::endl;
          float cos_theta_x = (point2[0] - point1[0])/magnitude;
          float cos_theta_y = (point2[1] - point1[1])/magnitude;
          float cos_theta_z = (point2[2] - point1[2])/magnitude;
                    /*
          float sin_theta_x=pow((1-pow(cos_theta_x,2)),0.5);
          float sin_theta_y=pow((1-pow(cos_theta_y,2)),0.5);
          float sin_theta_z=pow((1-pow(cos_theta_z,2)),0.5);

          //float R_x[3][3]={{1, 0, 0}, {0, cos_theta_x, -1*sin_theta_x}, {0, sin_theta_x, cos_theta_x}};
          //float R_y[3][3]={{cos_theta_y, 0, sin_theta_y}, {0, 1, 0}, {-1*sin_theta_y, 0, cos_theta_y}};
          //float R_z[3][3]={{cos_theta_z, -1*sin_theta_z, 0}, {sin_theta_z, cos_theta_z, 0}, {0, 0, 1}};
          //Rz*Ry*Rx
          
          tf1.setRotation(Matrix3f(cos_theta_y*cos_theta_z, (cos_theta_z*sin_theta_x*sin_theta_y)-(cos_theta_x*sin_theta_z), (cos_theta_x*cos_theta_z*sin_theta_y)+(sin_theta_x*sin_theta_z),
                   cos_theta_y*sin_theta_z,(cos_theta_x*cos_theta_z)+(sin_theta_x*sin_theta_y*sin_theta_z), (-1*cos_theta_z*sin_theta_x)+(cos_theta_x*sin_theta_y*sin_theta_z),
                   -1*sin_theta_y, cos_theta_y*sin_theta_x, cos_theta_x*cos_theta_y));
          
          
          //tf::Vector3 up_vector(0.0, 0.0, 1.0);
          tf::Vector3 axis_vector((point2[0]-point1[0]),(point2[1]-point1[1]),(point2[2]-point1[2]));
          std::cout<<(point2[0]-point1[0])<<"  "<<(point2[1]-point1[1])<<"  "<<(point2[2]-point1[2])<<std::endl;
          axis_vector.normalize();
          tf::Vector3 half(up_vector[0]+axis_vector[0], up_vector[1]+axis_vector[1], up_vector[2]+axis_vector[2]);
          tf::Vector3 right_vector = half.cross(up_vector);
          tf::Quaternion tf_q(right_vector[0],right_vector[1],right_vector[2],half.dot(up_vector));
          tf_q.normalize();
          */
          typedef Eigen::Matrix<float, 3, 1> Vector3f; 
          Vector3f up_vector(0.0, 0.0, 1.0);
          Vector3f axis_vector((point2[0]-point1[0]),(point2[1]-point1[1]),(point2[2]-point1[2]));
          q.setFromTwoVectors(up_vector,axis_vector);

         
          tf1.setQuatRotation(Quaternion3f(q.w(),q.x(),q.y(),q.z()));

          std::shared_ptr<Cylinder> Cylinder1(new Cylinder(0.03,magnitude));  //radius,length of collision object
          CollisionObject co1(Cylinder1, tf1);
          AABB a    = co1.getAABB() ;
          Vec3f vec =  a.center() ;
              drawSphere(vec);

          for(size_t i = 0; i < boxes.size(); ++i)
        {
            //std::cout<<i<<std::endl;
            CollisionObject* box =  boxes[i];
            static const int num_max_contacts = std::numeric_limits<int>::max();
            static const bool enable_contact = true ;
            fcl::CollisionResult result;
            fcl::CollisionRequest request(num_max_contacts, enable_contact);
            fcl::collide(&co1, box, request, result);

            if (result.isCollision() == true )
            {
                
                //collisionFlag.data = true ;
                //collisionFlagPub.publish(collisionFlag);
                std::cout<<"******Collision Detected"<<std::endl;
                collision_flag2=true;
                break;
            }
            else
            {
                //std::cout<<"******Line is safe"<<std::endl;
                collision_flag2=false;
            }
        }
  
      return collision_flag2;

}

std::vector<geometry_msgs::Pose> SimplifyPlan(std::vector<geometry_msgs::Pose> rrt_path)
{
  int initial_size=rrt_path.size(); // Repeat n times as many as the number of waypoints
  std::cout<<"initial Size is :"<<initial_size<<std::endl;
  int iterations=0;
  //int length=10;
  while(iterations!=initial_size)
  //for(int j=0;j<length;j++)
  {   

      iterations++;
      int length=rrt_path.size();
      //std::cout<<"New Size is :"<<length<<std::endl;
      int u;
      //for(int u=0;u<length;u++)
       // std::cout<<float(rrt_path[u].position.x)<<" "<<float(rrt_path[u].position.y)<<" "<<float(rrt_path[u].position.z)<<std::endl;
      std::random_device rd1;
      std::mt19937 gen1(rd1());
  
      std::uniform_real_distribution<> x(0, length);
      std::uniform_real_distribution<> y(0, length);
      int rand_indice1=int(x(gen1));
      int rand_indice2=int(y(gen1));
     std::cout<<"Random Indices are: "<<rand_indice1<<" "<<rand_indice2<<std::endl; 
      if(rand_indice1==rand_indice2)
      {
        continue;
      }
      else if (length==2)
      {
        break;
        //std::cout<<"Quitting Loop"<<std::endl;
      }
      else if(abs(rand_indice1-rand_indice2)==1)
      {
        continue;
      }
      
      else
      { 
        
        std::vector<float> path_point1,path_point2;
        path_point1.push_back(rrt_path[rand_indice1].position.x);path_point1.push_back(rrt_path[rand_indice1].position.y);path_point1.push_back(rrt_path[rand_indice1].position.z);
        path_point2.push_back(rrt_path[rand_indice2].position.x);path_point2.push_back(rrt_path[rand_indice2].position.y);path_point2.push_back(rrt_path[rand_indice2].position.z);
        bool linear_flag=isLineValid(path_point1,path_point2);
        if(!linear_flag)
            {
             // std::cout<<"Deleting points between "<<rand_indice1<<"and"<<rand_indice2<<std::endl;
              //delete elements between the two indices
              if(rand_indice1>rand_indice2)
              {
                int temporary=rand_indice1;
                rand_indice1=rand_indice2;
                rand_indice2=temporary;
              }
              /*
              auto it=rrt_path.begin()+rand_indice1;
              int temporary=rand_indice1+1;
              while(temporary<rand_indice2)
              {
                it++;
                temporary++;
                rrt_path.erase(it);
              }
              */
              int k = rand_indice2 - rand_indice1-1;

                auto it = rrt_path.cbegin() + rand_indice1+1;
                while (it != rrt_path.cend() && k--) {
                    it = rrt_path.erase(it);
                                 }

            }
      }
  }
          //Visualize Simplified Plan
          int length=rrt_path.size();
          std::cout<<"Number of Waypoints in Simplified Trajectory :"<<length<<std::endl;
          for(int h=0;h<length;h++)
          {
                simplified_trajectory.points.push_back(rrt_path[h].position);
            
                simplified_trajectory.points.push_back(rrt_path[h+1].position);
                pub5.publish(simplified_trajectory); 
                
         }
         return rrt_path;
}

std::vector<geometry_msgs::Pose> PrunePlan(std::vector<geometry_msgs::Pose> rrt_path)
{
  int initial_size=rrt_path.size(); 
  std::cout<<"initial Size is :"<<initial_size<<std::endl;
  geometry_msgs::Pose waypoint;
  //Adding first point to pruned path
  waypoint.position.x=rrt_path[0].position.x;
  waypoint.position.y=rrt_path[0].position.y;
  waypoint.position.z=rrt_path[0].position.z;
  pruned_path.push_back(waypoint);

  int current_index=0;
  pruned_size=pruned_path.size();
  std::vector<float> path_point1,path_point2;
  path_point1.push_back(rrt_path[0].position.x);path_point1.push_back(rrt_path[0].position.y);path_point1.push_back(rrt_path[0].position.z);
  
  //loop until goal is not added to pruned path
  while((pruned_path[pruned_size-1].position.x)!=(rrt_path[initial_size-1].position.x))
  {

      
      
      for(int iter=initial_size-1;iter>current_index;iter--)
      {
        path_point2.clear();
        path_point2.push_back(rrt_path[iter].position.x);path_point2.push_back(rrt_path[iter].position.y);path_point2.push_back(rrt_path[iter].position.z);
        std::cout<<"Currently checking: "<<current_index<<"and"<<iter<<std::endl;
        bool linear_flag=isLineValid(path_point1,path_point2);

        if(!linear_flag)
        {
          //line is safe add point to pruned path
          std::cout<<"Adding waypoint to pruned path"<<std::endl;
          waypoint.position.x=rrt_path[iter].position.x;
          waypoint.position.y=rrt_path[iter].position.y;
          waypoint.position.z=rrt_path[iter].position.z;
          pruned_path.push_back(waypoint);
          path_point1.clear();
          path_point1.push_back(rrt_path[iter].position.x);path_point1.push_back(rrt_path[iter].position.y);path_point1.push_back(rrt_path[iter].position.z);

          current_index=iter;
          break;

        }
        else
          continue;

      }


      //update pruned path size
      pruned_size=pruned_path.size();
  }


           //Visualize Pruned Plan
          int length=pruned_path.size();
          std::cout<<"Number of Waypoints in Pruned Trajectory :"<<length<<std::endl;
          for(int h=0;h<length;h++)
          {     
                if(h==0)
                {
               
                  simplified_trajectory.points.push_back(pruned_path[h].position);
        
                  simplified_trajectory.points.push_back(pruned_path[h+1].position);
                  pub5.publish(simplified_trajectory); 
                }
                else
                {
                    simplified_trajectory.points.push_back(pruned_path[h-1].position);
        
                  simplified_trajectory.points.push_back(pruned_path[h].position);
                  pub5.publish(simplified_trajectory); 

                }
                
         }
         return pruned_path;


}

std::vector<geometry_msgs::Pose> SmoothenPlan(std::vector<geometry_msgs::Pose> rrt_path)
{

std::vector<geometry_msgs::Pose> smooth_path;
geometry_msgs::Pose container;
int length=rrt_path.size();
std::cout<<"Path before smoothing has :"<<length<<std::endl;
    if(length>2)
    {
      //Bezier Curve with three points at a time
      for(int o=0;o<length-2;o++)
      {
        std::cout<<"Smoothing Path"<<std::endl;
        typedef Eigen::Matrix<float, 3, 1> Vector3f; 
        Vector3f pnt_1(rrt_path[o].position.x, rrt_path[o].position.y, rrt_path[o].position.z);
        Vector3f pnt_2(rrt_path[o+1].position.x, rrt_path[o+1].position.y, rrt_path[o+1].position.z);
        Vector3f pnt_3(rrt_path[o+2].position.x, rrt_path[o+2].position.y, rrt_path[o+2].position.z);
        //q.setFromTwoVectors(up_vector,axis_vector);
        /*
        Vector3f control_1,control_2,control_3,control_4;
        Vector3f u1=pnt_2-pnt_1;
        u1.normalize();
        control_1=pnt_2+(0.5*u1);                                             
        control_2=pnt_2+((0.5*u1)/2);
        Vector3f u2=pnt_3-pnt_2;
        u2.normalize();
        control_3=pnt_2-((0.5*u2)/2);
        control_4=pnt_2-(0.5*u2);
        */
        Vector3f control_1(pnt_2[0]*2-((pnt_1[0]+pnt_3[0])/2),
                          pnt_2[1]*2-((pnt_1[1]+pnt_3[1])/2),
                          pnt_2[2]*2-((pnt_1[2]+pnt_3[2])/2));    //control point for the bezier curve

        int idee=1;
        for(float t=0;t<1.1;t=t+0.1)
        {
          //std::cout<<t<<std::endl;
          //container.position.x=pow(1-t,3)*pnt_1[0]+pow(1-t,2)*3*t*control_3[0]+(1-t)*3*t*t*control_4[0]+t*t*t*pnt_2[0];
          //container.position.y=pow(1-t,3)*pnt_1[1]+pow(1-t,2)*3*t*control_3[1]+(1-t)*3*t*t*control_4[1]+t*t*t*pnt_2[1];
          //container.position.z=pow(1-t,3)*pnt_1[2]+pow(1-t,2)*3*t*control_3[2]+(1-t)*3*t*t*control_4[2]+t*t*t*pnt_2[2];
          container.position.x=pow(1-t,2)*pnt_1[0]+(1-t)*2*t*control_1[0]+t*t*pnt_3[0];
          container.position.y=pow(1-t,2)*pnt_1[1]+(1-t)*2*t*control_1[1]+t*t*pnt_3[1];
          container.position.z=pow(1-t,2)*pnt_1[2]+(1-t)*2*t*control_1[2]+t*t*pnt_3[2];
          //std::cout<<container.position.x<<" "<<container.position.y<<" "<<container.position.z<<std::endl;
          smooth_path.push_back(container);
          
          //send path to EGM
          container.orientation = tf::createQuaternionMsgFromYaw(0);
          trajectory_final.poses.push_back(container);

          //visualize path
          pnt_temp.pose.position.x=container.position.x;
          pnt_temp.pose.position.y=container.position.y;
          pnt_temp.pose.position.z=container.position.z;
          pnt_temp.id=idee;
          points_smooth.markers.push_back(pnt_temp);
          pub6.publish(points_smooth) ;
          idee++;

        }


        
      }

    }
    else
    {

      //Straight line so only Downsample it
      std::cout<<"Downsampling path"<<std::endl;
      typedef Eigen::Matrix<float, 3, 1> Vector3f; 
      Vector3f pnt_1(rrt_path[0].position.x, rrt_path[0].position.y, rrt_path[0].position.z);
      Vector3f pnt_2(rrt_path[1].position.x, rrt_path[1].position.y, rrt_path[1].position.z);
      Vector3f u1=pnt_2-pnt_1;
      //u1.normalize();
      int idee=1;
        for(float t=0;t<1.1;t=t+0.1)
        {

          Vector3f traverse=pnt_1+(u1*t);
          container.position.x= traverse[0];
          container.position.y= traverse[1];
          container.position.z= traverse[2];
          //std::cout<<container.position.x<<" "<<container.position.y<<" "<<container.position.z<<std::endl;
          smooth_path.push_back(container);

          //send path to EGM
          container.orientation = tf::createQuaternionMsgFromYaw(0);
          trajectory_final.poses.push_back(container);

          //visualize path
          pnt_temp.pose.position.x=container.position.x;
          pnt_temp.pose.position.y=container.position.y;
          pnt_temp.pose.position.z=container.position.z;
          pnt_temp.id=idee;
          points_smooth.markers.push_back(pnt_temp);
          pub6.publish(points_smooth) ;
          idee++;

        }


    }

    std::cout<<"Number of waypoints in the final trajectory:"<<smooth_path.size()<<std::endl;
    return smooth_path;

}

int main(int argc, char **argv)
{
    unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
   MTRand_int32 irand(init, length); // 32-bit int generator
// this is an example of initializing by an array
// you may use MTRand(seed) with any 32bit integer
// as a seed for a simpler initialization
  MTRand drand; // double in [0, 1) generator, already init

  
  ros::init(argc, argv, "rrt_3d");

  ros::NodeHandle nh;

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("rrt_3d_shapes", 10);
  ros::Publisher pub2 = nh.advertise<visualization_msgs::MarkerArray>("rrt_3d_random_points", 10);
  pub3=  nh.advertise<visualization_msgs::Marker>("rrt_3d_trajectory", 10);
  ros::Publisher pub4 = nh.advertise<geometry_msgs::PoseArray>("final_rrt_trajectory", 10);
  pub5=  nh.advertise<visualization_msgs::Marker>("rrt_simplified_trajectory", 10);
  ros::Subscriber sub = nh.subscribe("/moveit_cartesian_plan_plugin/feedback", 1000, Callback);
  ros::Subscriber sub2 = nh.subscribe("/octomap_full", 1000, octomap_Callback);
  visPub = nh.advertise<visualization_msgs::Marker>("Sphere", 1);
  pub6 = nh.advertise<visualization_msgs::MarkerArray>("rrt_smoothened_path", 10);
  ros::Rate loop_rate(20);
  

  //visualizations  points and lines..
//points.header.frame_id=mapData.header.frame_id;
//line.header.frame_id=mapData.header.frame_id;
  points_temp.header.frame_id = line_temp.header.frame_id= line_trajectory.header.frame_id  = simplified_trajectory.header.frame_id =pnt_temp.header.frame_id="world";
  points_temp.header.stamp=pnt_temp.header.stamp=ros::Time::now();
  line_temp.header.stamp=ros::Time::now();
  line_trajectory.header.stamp=ros::Time::now();
	
  points_temp.ns=line_temp.ns = line_trajectory.ns=pnt_temp.ns= "markers";

  line_temp.id =1;
  

  points_temp.type = shape;
  line_temp.type=line_trajectory.type= line_temp.LINE_LIST;

//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
points_temp.action =visualization_msgs::Marker::ADD;
pnt_temp.action =visualization_msgs::Marker::ADD;
line_temp.action = line_trajectory.action= visualization_msgs::Marker::ADD;
points_temp.pose.orientation.w =1.0;
//line_temp.pose.orientation.w = 1.0;
line_temp.scale.x =  0.02;
line_temp.scale.y= 0.02;
line_temp.scale.z= 0.02;
points_temp.scale.x=0.04; 
points_temp.scale.y=0.04; 
points_temp.scale.z=0.04;

line_temp.color.r =255.0/255.0;
line_temp.color.g= 0.0/255.0;
line_temp.color.b =0.0/255.0;
points_temp.color.r = 255.0/255.0;
points_temp.color.g = 0.0/255.0;
points_temp.color.b = 0.0/255.0;
points_temp.color.a=1;
line_temp.color.a = 1.0;
points_temp.lifetime = ros::Duration();
pnt_temp.lifetime=ros::Duration();
line_temp.lifetime = ros::Duration();
line_trajectory=simplified_trajectory=line_temp;

line_trajectory.color.r =0.0/255.0;
line_trajectory.color.g= 255.0/255.0;
line_trajectory.color.b =0.0/255.0;
line_trajectory.id =2;

simplified_trajectory.color.r =255.0/255.0;
simplified_trajectory.color.g= 255.0/255.0;
simplified_trajectory.color.b =255.0/255.0;
simplified_trajectory.id =3;

pnt_temp=points_temp;
pnt_temp.color.r = 255.0/255.0;
pnt_temp.color.g = 255.0/255.0;
pnt_temp.color.b = 255.0/255.0;
pnt_temp.color.a=1;
pnt_temp.id =4;
pnt_temp.type=shape2;
pnt_temp.scale.x=0.07; 
pnt_temp.scale.y=0.07; 
pnt_temp.scale.z=0.07;

/*
std::vector<geometry_msgs::Pose> testing;
geometry_msgs::Pose lite;
lite.position.x=0;
lite.position.y=0;
lite.position.z=2.5;
testing.push_back(lite);
lite.position.x=0;
lite.position.y=0;
lite.position.z=4;
testing.push_back(lite);
/*
lite.position.x=1;
lite.position.y=1;
lite.position.z=4;
testing.push_back(lite);

SmoothenPlan(testing);
//pub6.publish(points_smooth) ;
while(ros::ok())
{
pub6.publish(points_smooth) ;
}

*/

//wait until start and goal of tree is received
while(waiting_command&&ros::ok())
{
   ros::spinOnce();
   if(s&&g&&octomap_received)
      waiting_command=false;

}

//collision_result=isLineValid(start_command,goal_command);
//std::cout<<collision_result<<std::endl;
//while(ros::ok())

//Initial Start of Tree
points_temp.pose.position.x=start_command[0];                                                   // PARAMETER 4
points_temp.pose.position.y=start_command[1];
points_temp.pose.position.z=start_command[2];

//goal of the tree
goal_x=goal_command[0];                                                                         //PARAMETER 5
goal_y=goal_command[1];
goal_z=goal_command[2];
//pub.publish(points) ;

Xstartx=points_temp.pose.position.x;
Xstarty=points_temp.pose.position.y;
Xstartz=points_temp.pose.position.z;

//dimensions of the workspace                                                                     //PARAMETER 6
data.setFileName("/home/indraneel/.openrave/robot.cc9b336cd2b1b6a59d7902d01bc385db/reachability.6caac5769b42bdc42c377c1882948bec.pp"); // Set file name (only need once)
data.setVarName("reachabilitystats"); // Set variable name
std::vector<std::vector<double>> loadedData = data.getData(); // Load the data
//std::cout<<int(data.getSize())<<std::endl;
//std::cout<<double(loadedData[4][0])<<std::endl;


init_map_x= map_width=  5;
init_map_y= map_length= 5;
init_map_z= map_height= 10;

std::vector<float> point_rand,point_nearest,point_new,goal;

//Maintain list of vertices in the tree as array(x,y,z,parent_index)
std::vector< std::vector<float>> V; 

point_new.push_back( Xstartx);point_new.push_back( Xstarty);point_new.push_back(Xstartz);point_new.push_back(-1);  
//std::cout<<point_new[0]<<""<<point_new[1]<<""<<point_new[2]<<std::endl;
V.push_back(point_new);

goal.push_back( goal_x);goal.push_back( goal_y);goal.push_back(goal_z);
ros::Time FunctionStart = ros::Time::now();
while(ros::ok()&& goal_search)
{
      points_temp.id = i;
      //lines_temp.id=j;
      j=j+1;
      i=i+1;
      
      // Sample free
      point_rand.clear();
      point_new.clear();

      std::random_device rd;
      std::mt19937 gen(rd());
       //float map_width = costmap_->getSizeInMetersX();
      //float map_height = costmap_->getSizeInMetersY();
      //std::uniform_real_distribution<> x(-map_width, map_width);
      //std::uniform_real_distribution<> y(-map_length, map_length);
      //std::uniform_real_distribution<> z(0, map_height);
      std::uniform_real_distribution<> x(0, 7246479);
      random_indice=int(x(gen));
      //xr = x(gen);
      //yr = y(gen);
      //zr=  z(gen);
      xr=loadedData[random_indice][4];
      yr=loadedData[random_indice][5];
      zr=loadedData[random_indice][6];

              

      point_rand.push_back( xr ); point_rand.push_back( yr ); point_rand.push_back( zr );
      
          
      //collision_result=true;
      std::cout<<point_rand[0]<<"  "<<point_rand[1]<<"  "<<point_rand[2]<<std::endl;
      // Nearest
      std::tie(point_nearest,parent_index)=Nearest(V,point_rand);
      //std::cout<<parent_index<<std::endl;
      //*************EXTEND FUNCTION**************************
      //Check if within limits of max travel distance
      if(Norm(point_nearest,point_rand)<=eta)
      {
              point_new=point_rand;
              point_new.push_back(parent_index);
              found=true;
      }
      else

      {
          found=false;
           // proceed in the direction of the randomly sampled point from nearest point
          float magnitude =pow(pow(point_rand[2] - point_nearest[2],2)+pow(point_rand[1] - point_nearest[1],2)+pow(point_rand[0] - point_nearest[0],2),0.5);
          
          float cos_theta_x = (point_rand[0] - point_nearest[0])/magnitude;
          float cos_theta_y = (point_rand[1] - point_nearest[1])/magnitude;
          float cos_theta_z = (point_rand[2] - point_nearest[2])/magnitude;
      

           point_new.push_back(point_nearest[0] + eta * cos_theta_x);
           point_new.push_back(point_nearest[1] + eta * cos_theta_y);
           point_new.push_back(point_nearest[2] + eta * cos_theta_z);
           point_new.push_back(parent_index);
           
  /*      
          float m=(point_rand[1]-point_nearest[1])/(point_rand[0]-point_nearest[0]);

          point_new.push_back(  (sign(point_rand[0]-point_nearest[0]))* (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   )+point_nearest[0] );
          point_new.push_back(  m*(point_new[0]-point_nearest[0])+point_nearest[1] );

          if(point_rand[0]==point_nearest[0]){
                point_new[0]=point_nearest[0];
                point_new[1]=point_nearest[1]+eta;

                      }*/

      }
//if(found==true)
            collision_result=isStateValid(point_new);
            if(!collision_result)
            {
                V.push_back(point_new);
                p.x=point_new[0]; 
                p.y=point_new[1]; 
                p.z=point_new[2];
                line_temp.points.push_back(p);
                p.x=point_nearest[0]; 
                p.y=point_nearest[1]; 
                p.z=point_nearest[2];
                line_temp.points.push_back(p);
                pub.publish(line_temp); 

                //check if we have reached our goal

                if(Norm(goal,point_new)<0.4)                                                 // PARAMETER 7
                { 
                    std::cout<<"We have reached our goal"<<std::endl;
                    ros::Time FunctionEnd = ros::Time::now();
                    std::cout<<"\n Function Took:"<<FunctionEnd - FunctionStart;
                    goal.push_back(V.size()-1);   //Add parent index for goal
                    V.push_back(goal);
                    //int goal_vertex = V.get_index();
                    p.x=point_new[0]; 
                    p.y=point_new[1]; 
                    p.z=point_new[2];
                    //line_trajectory.points.push_back(p); 
                    line_temp.points.push_back(p);
                    p.x=goal[0]; 
                    p.y=goal[1]; 
                    p.z=goal[2];
                    line_temp.points.push_back(p);
                    //line_trajectory.points.push_back(p);
                    pub.publish(line_temp); 
                    //pub3.publish(line_trajectory);
                    goal_search=false;

                    plan=BuildPlan(V);
                    simplify_plan=PrunePlan(plan);
                    smooth_plan=SmoothenPlan(simplify_plan);
                    std::cout << "Rows in the 2d vector: " << V.size() <<std::endl << "Collumns in the 1st row: " << V[0].size() <<std::endl;
                    int h=V.size();
                    int l=V[h-1][3];
                    trajectory_final.header.stamp=ros::Time::now();
                    trajectory_final.header.frame_id="world";
                    trajectory_final.header.seq++;
                    //Send to EGM
                    pub4.publish(trajectory_final);

                      //std::cout<<V[h-1][3]<<std::endl;
                    //std::cout<<V[l][3]<<std::endl;

                  }
            }
            points_temp.pose.position.x=point_rand[0];
            points_temp.pose.position.y=point_rand[1];
            points_temp.pose.position.z=point_rand[2];


            points.markers.push_back(points_temp);
            pub2.publish(points) ;
            //points.points.clear();
            loop_rate.sleep();
}

/*

  int count = 0;
  while (ros::ok())
  {
  
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
*/

  return 0;
}

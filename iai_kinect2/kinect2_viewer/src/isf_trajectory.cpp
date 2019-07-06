#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <kinect2_bridge/kinect2_definitions.h>
#include <std_msgs/Float32.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_config.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/shot_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
using namespace std;


using vec = vector<float>;
using matrix = vector<vec>;



int a=0;
int t=0;
pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());


matrix readCSV( string filename )
{
   char separator = ',';
   matrix result;
   string row, item;

   ifstream in( filename );
   while( getline( in, row ) )
   {
      vec R;
      stringstream ss( row );
      while ( getline ( ss, item, separator ) ) R.push_back( std::stoi(item) );
      result.push_back( R );
   }
   in.close();
   return result;
}

void printMatrix( const matrix &M )
{
  for (int i = 0; i < 29; i++){

    cout<<M[1][2]<<endl;

}
}


void push_back()
{

  matrix ISF = readCSV( "c_exp1.csv" );
  printMatrix( ISF );

  if(a==0)
  {

    for (int i = 0; i < 29; i++)
  {
    pcl::PointXYZ point;
          
    point.x = ISF[i][0];
    point.y = ISF[i][1];
    point.z = ISF[i][2];  ;
    particle_cloud->points.push_back (point);
    std::cout << "Point sent to cloud: " << std::endl;
  }
  a=a+1;
  }

}



void cloudViewer(const std_msgs::Float32ConstPtr& number)
    {

      

        if(t==0)
         {
          OUT_INFO("In cloud viewer");
          const std::string cloudName = "rendered";
   
          push_back();
          OUT_INFO("I'm Back!!");
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> points_colour (particle_cloud, 255, 0, 0);

          visualizer->addPointCloud(particle_cloud,points_colour, cloudName);
          //visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
          //visualizer->initCameraParameters();
          visualizer->setBackgroundColor(0, 0, 0);
          //visualizer->setShowFPS(true);
          //visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
          t=t+1;
        }

          visualizer->spinOnce(5);

    }




int
main (int argc, char** argv)
{ 
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub1 = nh.subscribe ("/kinect2/sd/image_ir_rect", 1000, cloud_cb1);
  //ros::Subscriber sub2 = nh.subscribe ("/kinect2/sd/image_depth_rect", 1000, cloud_cb2);
  //ros::Subscriber sub3 = nh.subscribe ("/kinect2/sd/camera_info", 1000, cloud_cb3);
  //ros::Subscriber sub4 = nh.subscribe ("/kinect2/sd/camera_info", 1000, cloud_cb4);
  
  ros::Subscriber sub5 = nh.subscribe ("invoke_visualizer", 1000, cloudViewer);
  //ros::Subscriber sub6 = nh.subscribe ("push_back", 1000, push_back);
  ros::spin ();
}
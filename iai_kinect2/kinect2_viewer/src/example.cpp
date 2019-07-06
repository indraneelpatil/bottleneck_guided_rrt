#include <pcl/filters/voxel_grid.h>
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
#include <time.h>
#include <std_msgs/Float32.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/filters/model_outlier_removal.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/region_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>



int i=0;
int z=0;

cv::Mat color_img, depth_img;
cv::Mat lookupX, lookupY;
cv::Mat cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
cv::Mat cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);

sensor_msgs::CameraInfo cameraInfoColor;
sensor_msgs::CameraInfo cameraInfoDepth;

pcl::PointCloud<pcl::PointXYZRGBA> boundary_cloud;
pcl::PointCloud<pcl::PointXYZRGBA> test;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered2 (&test);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundary_cloud_ptr (&boundary_cloud);

pcl::PointXYZRGBA centre;
Eigen::Vector4f centroid; 
ros::Publisher chatter_pub;

pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
const std::string cloudName = "rendered";

pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg; 
  pcl::NormalEstimation<pcl::PointXYZRGBA,pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal>);

  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label > mps;
  std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > regions;
 


int a=0;
int cloud_initialise=0;

void 
cloud_cb1 (const sensor_msgs::ImageConstPtr& image_store)
{   
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(image_store, image_store->encoding);
    pCvImage->image.copyTo(color_img);
    OUT_INFO("found color image...");
    if(color_img.type() == CV_16U)
    {
      cv::Mat tmp;
      color_img.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color_img, CV_GRAY2BGR);
    }
}


void 
cloud_cb2 (const sensor_msgs::ImageConstPtr& depth_store)
{
    cv_bridge::CvImageConstPtr pCvImage2;
    pCvImage2 = cv_bridge::toCvShare(depth_store, depth_store->encoding);
    pCvImage2->image.copyTo(depth_img);
    OUT_INFO("found depth image...");
}

void 
cloud_cb3 (const sensor_msgs::CameraInfoConstPtr& cameraInfo1)
{
    
    double *itC = cameraMatrixColor.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo1->K[i];
    }
     cameraInfoColor=*cameraInfo1;
}
   
void 
cloud_cb4 (const sensor_msgs::CameraInfoConstPtr& cameraInfo2)
{

    double *itC = cameraMatrixDepth.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo2->K[i];
    }
     cameraInfoDepth=*cameraInfo2;
}


void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;
    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
}

void createcloud ()
{
  if(cloud_initialise==0)
  {
     cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    
    cloud->height = color_img.rows;
    cloud->width = color_img.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(color_img.cols, color_img.rows);
    cloud_initialise=cloud_initialise+1;
   } 

    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth_img.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth_img.cols];
      const uint16_t *itD = depth_img.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color_img.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();
      for(size_t c = 0; c < (size_t)depth_img.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(*itD == 0)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
}




void processing ()
{
  

  

  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.8);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

pcl::PassThrough<pcl::PointXYZRGBA> pass1;
  pass1.setInputCloud (cloud_filtered);
  pass1.setFilterFieldName ("x");
  pass1.setFilterLimits (0.0, 0.8);
  //pass.setFilterLimitsNegative (true);
  pass1.filter (*cloud_filtered);

 
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setLeafSize (0.005, 0.005, 0.005);  //0.01  //0.005   //plane 0.02
  sor.filter (*cloud_filtered2);

  
/*
 pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
// Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud (cloud_filtered2);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

*/


      pcl::io::savePCDFileASCII ("sphere_downsample.pcd", *cloud_filtered2);
      std::cerr << "Saved " << cloud_filtered2->points.size () << " data points to sphere_pcd.pcd." << std::endl;
  

  

  

/*

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers, *coefficients);

   // Extract the inliers
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_filtered3);

  pcl::ModelOutlierRemoval<pcl::PointXYZRGBA> filter;
  filter.setModelCoefficients (*coefficients);
  filter.setThreshold (0.08);
  filter.setModelType (pcl::SACMODEL_PLANE);
  filter.setInputCloud (cloud_filtered3);
  filter.filter (*cloud_plane_filtered);

 
  pcl::compute3DCentroid(*cloud_plane_filtered,centroid); 
  centre.x= centroid[0];
  centre.y=centroid[1];
  centre.z=centroid[2];
  centre.r=0;
  centre.g=255;
  centre.b=0;
  cout<<centroid<<endl;
  cloud_plane_filtered->points.push_back(centre);
*/
/*
  for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it=cloud_plane_filtered->begin() ;it!=cloud_plane_filtered->end() ;it++)
  {

    i=i+1;
    cout<<i<< "__Points are : "<<it->x <<it->y <<it->z<< endl;

  }

  i=0;
  */
/*
  seg.setInputCloud(cloud_filtered2);
  seg.setInputNormals(cloud_normals);
  seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02); 
  seg.setOptimizeCoefficients(true);
  seg.setRadiusLimits(0, 0.06);
  seg.setEpsAngle(40 / (180/3.141592654));
  seg.setMaxIterations(1000000);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers, *coefficients);

*/


/*
  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
 


  std::cerr << "Model inliers: " << inliers_cylinder->indices.size () << std::endl;
  for (size_t i = 0; i < inliers_cylinder->indices.size (); ++i)
  {
    std::cerr << inliers_cylinder->indices[i] << "    " << cloud->points[inliers_cylinder->indices[i]].x << " "
                                               << cloud->points[inliers_cylinder->indices[i]].y << " "
                                               << cloud->points[inliers_cylinder->indices[i]].z << std::endl;
    cloud_filtered2->points[inliers_cylinder->indices[i]].r = 255; 
    cloud_filtered2->points[inliers_cylinder->indices[i]].g = 0; 
    cloud_filtered2->points[inliers_cylinder->indices[i]].b = 0; 
   }


   std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
  {
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               << cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << std::endl;
    cloud_filtered2->points[inliers->indices[i]].r = 255; 
    cloud_filtered2->points[inliers->indices[i]].g = 0; 
    cloud_filtered2->points[inliers->indices[i]].b = 0; 
   }
 
   
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();
    chatter_pub.publish(msg);
    //  ros::spinOnce();

*/
  }



void outlier_removal()
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);


}

void cloudViewer(const std_msgs::Float32ConstPtr& number)
{
   if(a==0)
   {
   OUT_INFO("In cloud viewer");
   const std::string cloudName = "rendered";

   createcloud();
   processing();
   //outlier_removal();

   visualizer->addPointCloud(cloud_filtered2, cloudName);
   //visualizer-> addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal> (cloud_filtered2,cloud_normals,100,0.05,"normals");
   visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
   visualizer->initCameraParameters();
   visualizer->setBackgroundColor(0, 0, 0);
   visualizer->setSize(color_img.cols, color_img.rows);
   visualizer->setShowFPS(true);
   visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

  
   a=a+1;
   }

   createcloud();
   processing();
   //outlier_removal();
   
    

   visualizer->updatePointCloud(cloud_filtered2, cloudName);
   //visualizer->removePointCloud("normals");
   //visualizer-> addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal> (cloud_filtered2,cloud_normals,100,0.05,"normals");
   
   visualizer->spinOnce(5);

   OUT_INFO("Cloud name is: " FG_CYAN << cloudName << NO_COLOR);


}




int
main (int argc, char** argv)
{ 
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/kinect2/sd/image_ir_rect", 1000, cloud_cb1);
  ros::Subscriber sub2 = nh.subscribe ("/kinect2/sd/image_depth_rect", 1000, cloud_cb2);
  ros::Subscriber sub3 = nh.subscribe ("/kinect2/sd/camera_info", 1000, cloud_cb3);
  ros::Subscriber sub4 = nh.subscribe ("/kinect2/sd/camera_info", 1000, cloud_cb4);
  
  ros::Subscriber sub5 = nh.subscribe ("invoke_visualizer", 1000, cloudViewer);
  ros::spin ();
}


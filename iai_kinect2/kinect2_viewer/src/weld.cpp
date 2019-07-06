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
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/model_outlier_removal.h>
#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>

 Eigen::Vector4f centroid; 
 pcl::PointXYZRGBA centre;
 pcl::PointXYZRGBA to_push;
 pcl::PointXYZRGBA searchPoint;

int j=0;
int i=0;
int a=0;
int t=0;
int g=0;
int cloud_initialise=0;

cv::Mat color_img, depth_img;
cv::Mat lookupX, lookupY;
cv::Mat cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
cv::Mat cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);

sensor_msgs::CameraInfo cameraInfoColor;
sensor_msgs::CameraInfo cameraInfoDepth;

pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmented_edge (new pcl::PointCloud<pcl::PointXYZRGBA>); 
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); 
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundary_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);  
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgb_edges(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_point(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ef(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_end(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluded_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_out (new pcl::PointCloud<pcl::PointXYZRGBA>);

pcl::PointCloud<pcl::Normal>::Ptr cloud_filtered_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane_fin (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_fin (new pcl::PointCloud<pcl::PointXYZRGBA>);
//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
pcl::ExtractIndices<pcl::Normal> extract_normals;


const std::string cloudName = "rendered";

pcl::PointXYZRGBA centre2;
Eigen::Vector4f centroid2; 


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

   
       createcloud();



        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

           for (int i = 0; i < (*cloud_ef).size(); i++)
        {
            if (g==0) // e.g. remove all pts below zAvg
        {
            inliers->indices.push_back(i);
        }

        }



        extract.setInputCloud(cloud_ef);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_ef);

        pcl::PointIndices::Ptr inliers_z (new pcl::PointIndices);

           for (int i = 0; i < (*centroid_cloud).size(); i++)
        {
            if (g==0) // e.g. remove all pts below zAvg
        {
            inliers_z->indices.push_back(i);
        }

        }



        extract.setInputCloud(centroid_cloud);
        extract.setIndices(inliers_z);
        extract.setNegative(true);
        extract.filter(*centroid_cloud);


         pcl::PointIndices::Ptr inliers_p (new pcl::PointIndices);

           for (int i = 0; i < (*segmented_edge).size(); i++)
        {
            if (g==0) // e.g. remove all pts below zAvg
        {
            inliers_p->indices.push_back(i);
        }

        }



        extract.setInputCloud(segmented_edge);
        extract.setIndices(inliers_p);
        extract.setNegative(true);
        extract.filter(*segmented_edge);
        


        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 0.8);
        pass.filter (*cloud_filtered_out);
        std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
        std::cerr << "PointCloud after Pass through filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

   
        pcl::VoxelGrid<pcl::PointXYZRGBA> srm;
        srm.setInputCloud (cloud_filtered_out);
        srm.setLeafSize (0.01,0.01,0.01);
        srm.filter (*cloud_filtered);
    
        std::cerr << "PointCloud after Voxel filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

        pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> nes;
        nes.setKSearch (15);
        nes.setSearchMethod (tree);
        nes.setInputCloud (cloud_filtered);
        nes.compute (*cloud_filtered_normals);



            pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
            //pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_CYLINDER);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (0.1);
            seg.setMaxIterations (200);
            seg.setDistanceThreshold (0.05);
            seg.setRadiusLimits (0, 0.2);
            seg.setInputCloud (cloud_filtered);
            seg.setInputNormals (cloud_filtered_normals);
 
  // Obtain the cylinder inliers and coefficients
            seg.segment (*inliers_cylinder, *coefficients_cylinder);

  
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers_cylinder);
            extract.setNegative (false);
            extract.filter (*cloud_plane_fin);

            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
            sor.setInputCloud (cloud_plane_fin);
            sor.setMeanK (500);
            sor.setStddevMulThresh (1.0);
            sor.filter (*cloud_plane);

            pcl::compute3DCentroid(*cloud_plane,centroid); 

            //cout <<centroid <<endl;
            centre.x=centroid[0];
            centre.y=centroid[1];
            centre.z=centroid[2];
            centre.r=0;
            centre.g=0;
            centre.b=128;
            cloud_point->points.push_back(centre);
            cout<<centre<<endl;

     

        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setNormalSmoothingSize (10.0f);
        ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);
        ne.setInputCloud (cloud);
        ne.compute (*cloud_normals);

        pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> oed;
        oed.setInputNormals (cloud_normals);
        oed.setInputCloud (cloud);
        oed.setDepthDisconThreshold (0.02); // 2cm
        oed.setMaxSearchNeighbors (9);
        pcl::PointCloud<pcl::Label> labels;
        std::vector<pcl::PointIndices> label_indices;
    
        oed.compute (labels, label_indices);

        pcl::copyPointCloud (*cloud, label_indices[0].indices, *boundary_edges);
        pcl::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
        pcl::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
        pcl::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
        pcl::copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);

     std::cerr << "Scene PointCloud has: " << cloud->points.size () << " points." << std::endl;
     std::cerr << "RGB has: " << rgb_edges->points.size () << " points." << std::endl;
     std::cerr << "occluded has: " << occluding_edges->points.size () << " points." << std::endl;
     std::cerr << "High Curvature Edges has: " << high_curvature_edges->points.size () << " points." << std::endl;


     pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;

     kdtree.setInputCloud (cloud_plane);
     float K=0.1;  //0.02
     std::vector<int> pointIdxNKNSearch(K);
     std::vector<float> pointNKNSquaredDistance(K);

     for (size_t j = 0; j < rgb_edges->points.size (); ++j)
    {
      searchPoint.x = rgb_edges->points[j].x;
      searchPoint.y = rgb_edges->points[j].y;
      searchPoint.z = rgb_edges->points[j].z;
      std::cerr << "Edge point is : " << rgb_edges->points[j].x  << "  " << rgb_edges->points[j].y << "  " << rgb_edges->points[j].z << std::endl;

         if ( kdtree.radiusSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
           {
                for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){


                 to_push.x = cloud_plane->points[ pointIdxNKNSearch[i] ].x;
                to_push.y = cloud_plane->points[ pointIdxNKNSearch[i] ].y;
                to_push.z = cloud_plane->points[ pointIdxNKNSearch[i] ].z;
                cloud_ef->points.push_back(to_push);
                 cloud_end->points.push_back(to_push);

                 }
       
               pcl::compute3DCentroid(*cloud_end,centroid2); 
                centre2.x= centroid2[0];
                centre2.y=centroid2[1];
               centre2.z=centroid2[2];
               centre2.r=0;
               centre2.g=255;
               centre2.b=255;
               if(centroid2[0]- rgb_edges->points[j].x > 0.025)
               //centroid_cloud->points.push_back(centre2);
                segmented_edge->points.push_back(rgb_edges->points[j]);

                std::cerr << "Corresponding Centroid point is : " << centroid2[0] << "  " << centroid2[1] <<"  "<< centroid2[2] << std::endl;
                 std::cerr << "************************* "  << std::endl;
                 }


    
          pcl::PointIndices::Ptr inliers_t (new pcl::PointIndices);

                for (int i = 0; i < (*cloud_end).size(); i++)
        {
            if (g==0) // e.g. remove all pts below zAvg
        {
            inliers_t->indices.push_back(i);
        }

        }



        extract.setInputCloud(cloud_end);
        extract.setIndices(inliers_t);
        extract.setNegative(true);
        extract.filter(*cloud_end);

  



    }

    std::cerr << "All Edge PointCloud has: " << cloud_ef->points.size () << " points." << std::endl;
/*
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree2;

     kdtree2.setInputCloud (cloud_ef);
     float r=0.03f;
     std::vector<int> pointIdxRadiusSearch;
     std::vector<float> pointRadiusSquaredDistance;

     for (size_t j = 0; j < cloud_ef->points.size (); ++j)
    {
      searchPoint.x = cloud_ef->points[j].x;
      searchPoint.y = cloud_ef->points[j].y;
      searchPoint.z = cloud_ef->points[j].z;


     if ( kdtree2.radiusSearch (searchPoint, r, pointIdxRadiusSearch,pointRadiusSquaredDistance)  > 55)
    {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){


         to_push.x = cloud_plane->points[ pointIdxRadiusSearch[i] ].x;
         to_push.y = cloud_plane->points[ pointIdxRadiusSearch[i] ].y;
         to_push.z = cloud_plane->points[ pointIdxRadiusSearch[i] ].z;
         cloud_end->points.push_back(to_push);
    }   
    }
    }
    std::cerr << "Weld line has: " << cloud_end->points.size () << " points." << std::endl;
  */ 

}


void cloudViewer(const std_msgs::Float32ConstPtr& number)

{
   if(a==0)
   {
   OUT_INFO("In cloud viewer");
   const std::string cloudName = "rendered";

   
   processing();

   OUT_INFO("Back to viewer");

  // visualizer->addPointCloud (cloud, cloudName);
   //visualizer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud_filtered, cloud_filtered_normals, 100, 0.05, "normals");
 //  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cyl (cloud_plane, 0, 255, 0);
  // visualizer->addPointCloud (cloud_plane, cyl, "cylinder");
   
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> hce (cloud_ef, 255, 0, 0);
  // visualizer->addPointCloud (cloud_ef, hce, "k_edges");

   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgb (rgb_edges, 0, 0, 255);
   visualizer->addPointCloud (rgb_edges, rgb, "rgb_edges");


  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> temp (centroid_cloud, 0, 255, 255);
 //  visualizer->addPointCloud (centroid_cloud, temp, "centroid_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> temp2 (segmented_edge, 255, 255, 0);
   visualizer->addPointCloud (segmented_edge, temp2, "segmented_edge");

   //visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rgb_edges");
  // visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "high_curvature_edges");
   //visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);

   visualizer->initCameraParameters();
   visualizer->setBackgroundColor(0, 0, 0);
   visualizer->setShowFPS(true);
   visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
   a=a+1;
   }

   processing();
  // visualizer->updatePointCloud (cloud, cloudName);
   
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cyl (cloud_plane, 0, 255, 0);
  // visualizer->updatePointCloud (cloud_plane, cyl, "cylinder");
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgb (rgb_edges, 0, 0, 255);
  visualizer->updatePointCloud (rgb_edges, rgb, "rgb_edges");

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> hce (cloud_ef, 255, 0, 0);
  // visualizer->updatePointCloud (cloud_ef, hce, "k_edges");

 //  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> temp (centroid_cloud, 0, 255, 255);
 //  visualizer->updatePointCloud (centroid_cloud, temp, "centroid_cloud");

   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> temp2 (segmented_edge, 255, 255, 0);
   visualizer->updatePointCloud (segmented_edge, temp2, "segmented_edge");


   //visualizer->removePointCloud("normals");
   //visualizer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud_filtered, cloud_filtered_normals, 100, 0.05, "normals");
   visualizer->spinOnce(5);

   OUT_INFO("Cloud name is: " FG_CYAN << cloudName << NO_COLOR);

}


int
main (int argc, char** argv)
{ 
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/kinect2/sd/image_ir_rect", 1000, cloud_cb1);
  ros::Subscriber sub2 = nh.subscribe ("/kinect2/sd/image_depth_rect", 1000, cloud_cb2);
  ros::Subscriber sub3 = nh.subscribe ("/kinect2/sd/camera_info", 1000, cloud_cb3);
  ros::Subscriber sub4 = nh.subscribe ("/kinect2/sd/camera_info", 1000, cloud_cb4);
  
  ros::Subscriber sub5 = nh.subscribe ("invoke_visualizer", 1000, cloudViewer);
  ros::spin ();
}


/*
   for (size_t i = 0; i < cloud_plane->points.size (); ++i)

   {
    for (size_t j = 0; j < rgb_edges->points.size (); ++j)
    {

        if (cloud_plane->points[i].x == rgb_edges->points[j].x && cloud_plane->points[i].y == rgb_edges->points[j].y && cloud_plane->points[i].z == rgb_edges->points[j].z)
        {


          to_push.x = cloud_plane->points[i].x;
          to_push.y = cloud_plane->points[i].y;
          to_push.z = cloud_plane->points[i].z;

          cloud_ef->points.push_back(to_push);
        }

    }
    }
*/
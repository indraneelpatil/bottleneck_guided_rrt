#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/ColorOcTree.h>
#include <visualization_msgs/MarkerArray.h>

typedef octomap::ColorOcTree OcTreeT;
using namespace octomap;
using octomap_msgs::Octomap;

 OcTreeT* m_octree;
 octomap::KeyRay m_keyRay;  // temp storage for ray casting
 octomap::OcTreeKey m_updateBBXMin;
 octomap::OcTreeKey m_updateBBXMax;

 int count=0;

 //downprojected 2D Map
 nav_msgs::OccupancyGrid m_gridmap;
 unsigned m_treeDepth,m_maxTreeDepth;
 bool publishMarkerArray=true;
 bool publishFreeMarkerArray=true;
 std_msgs::ColorRGBA m_color;
  std_msgs::ColorRGBA m_colorFree;

inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
    for (unsigned i = 0; i < 3; ++i)
      min[i] = std::min(in[i], min[i]);
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };



int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_generator");
	ros::NodeHandle m_nh;
	ros::Publisher chatter_pub = m_nh.advertise<std_msgs::String>("chatter", 1000);
	  ros::Publisher m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 100);
      ros::Publisher m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 100);
       ros::Publisher m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1);
 	//ros::Rate loop_rate(10);

    ros::Time rostime = ros::Time::now();
    m_color.r = 0;
  	m_color.g = 0;
  	m_color.b = 1;
  	m_color.a = 1;

  	m_colorFree.r = 0;
  	m_colorFree.g = 1;
  	m_colorFree.b = 0;
  	m_colorFree.a = 1;


 	// initialize octomap object & params
  	m_octree = new OcTreeT(0.05);                //resolution of Octomap
  	m_octree->setProbHit(0.7);
  	m_octree->setProbMiss(0.4);
  	m_octree->setClampingThresMin(0.12);
  	m_octree->setClampingThresMax( 0.97);
  	
  	m_treeDepth=m_maxTreeDepth=1000;
  	//m_treeDepth = m_octree->getTreeDepth();
  	//m_maxTreeDepth = m_treeDepth;
  	//m_gridmap.info.resolution = m_res;

 	 ros::WallTime startTime = ros::WallTime::now();
 	 KeySet free_cells, occupied_cells;
 	 point3d sensorOrigin(0,0,0);
 	 point3d point(5, 5, 5);

       // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }

      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)){
        occupied_cells.insert(key);

        updateMinKey(key, m_updateBBXMin);
        updateMaxKey(key, m_updateBBXMax);

    }

     // mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      m_octree->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    m_octree->updateNode(*it, true);
  }

  size_t octomapSize = m_octree->size();
  std::cout<<octomapSize<<std::endl;
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return 0;
  }

  // init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(1000);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(1000);
  
  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
      end = m_octree->end(); it != end; ++it)
  {
    //bool inUpdateBBX = isInUpdateBBX(it);

count=count+1;
//std::cout<<count<<std::endl;

if (m_octree->isNodeOccupied(*it)){
      double z = it.getZ();
      //if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      //{
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
        std::cout<<x<<"  "<<y<<"  "<<z<<std::endl;
        		//create marker:
        if (publishMarkerArray){
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
          /*
          if (m_useHeightMap){
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }*/

      }
  }
  else{ // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      //if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      //{
     

        //if (m_publishFreeSpace){
          double x = it.getX();
          double y = it.getY();

          //create marker for free space:
          if (publishFreeMarkerArray){
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
       // }

      }
  //}

}



 // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = "world";
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      occupiedNodesVis.markers[i].color = m_color;
      occupiedNodesVis.markers[i].lifetime = ros::Duration();


      if (occupiedNodesVis.markers[i].points.size() > 0)
      {
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
       
    }
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    
    m_markerPub.publish(occupiedNodesVis);
  }

// finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = "world";
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map_free";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;
      freeNodesVis.markers[i].lifetime = ros::Duration();
      //freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
       std::cout<<"HEY"<<std::endl;

      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        
    }

    m_fmarkerPub.publish(freeNodesVis);
  }

while(ros::ok())
{
	m_fmarkerPub.publish(freeNodesVis);
	 m_markerPub.publish(occupiedNodesVis);
  ros::spinOnce();
}

/*
  Octomap map;
  map.header.frame_id = "world";
  map.header.stamp = rostime;

  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    m_fullMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
*/
 	 
 	 /*
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
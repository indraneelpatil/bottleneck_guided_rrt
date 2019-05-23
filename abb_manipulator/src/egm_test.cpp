#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <abb_libegm/egm_base_interface.h>
#include <abb_libegm/egm_common.h>
#include <abb_libegm/egm_common_auxiliary.h>
#include <abb_libegm/egm_controller_interface.h>
#include <abb_libegm/egm_interpolator.h>
#include <abb_libegm/egm_logger.h>
#include <abb_libegm/egm_server.h>
#include <abb_libegm/egm_trajectory_interface.h>
 #include <iostream>
 #include <thread>
 #include <mutex>
 #include <boost/thread.hpp>
#include "egm.pb.h"
#include "egm_wrapper.pb.h"
#include "egm_wrapper_trajectory.pb.h"


// Boost components (for managing asynchronous UDP sockets).
boost::asio::io_service io_service;
boost::thread_group worker_threads;

// EGM interface components (for setting up and providing APIs to a EGM server).
abb::egm::BaseConfiguration configuration;

abb::egm::EGMControllerInterface egm_interface(io_service, 6511 /* Port number (needs to match the robot controller EGM settings) */, configuration);


// Simple joint velocity control loop.
abb::egm::wrapper::Input input;
abb::egm::wrapper::Output output;

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
configuration.use_velocity_outputs = true;
worker_threads.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

while (ros::ok())
{
  if (egm_interface.waitForMessage(500))
  {
    egm_interface.read(&input);
    std::cout<<"Egm interface is :"<<std::endl;
    

    // Calculate, and set references. For example:
    output.Clear();
    output.mutable_robot()->mutable_joints()->mutable_velocity()->add_values(2.0);
    // Etc.

    egm_interface.write(output);
    std::cout<<"Data is written"<<std::endl;
  }
}

  return 0;

}
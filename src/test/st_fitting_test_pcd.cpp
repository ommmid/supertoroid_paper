#include <ros/ros.h>
#include <iostream>
#include <supertoroid/st_fitting.h>
#include <supertoroid/st.h>
#include <supertoroid/st_sampling.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int main(int argc, char *argv[])
{ 
  ros::init(argc, argv, "st_fitting_test_pcd");
  ros::NodeHandle nh("~");
  double a1_param;
  nh.getParam("/fittingST/a1param",a1_param);
  std::cout<<"a1 value: "<<a1_param<<std::endl;


  std::cout<<"I HAVE STARTED"<<std::endl;
  if(argc !=2) {  //command line arguments: name of program + point cloud (2)
    std::cout<<"I DONT SEE TWO ARGUMENTS"<<std::endl;
    return (0);
  }
  std::cout<<"I SEE TWO ARGUMENTS"<<std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  std::cout<<"I CREATED THE POINT CLOUD"<<std::endl;
  if(pcl::io::loadPCDFile(argv[1], *cloud)== -1) {
      std::cout<<"I COULD NOT READ THE POINT CLOUD"<<std::endl;
      return -1;
  }
  std::cout<<"Size of point cloud: "<<cloud->size()<<std::endl;

  Fitting* fit = new Fitting(cloud);
  supertoroid::st min_param;
  fit->fit();
  fit->getMinParams(min_param);


  std::cout<<"Minimum parameters are: "<<"a1:"<<min_param.a1<<"  a2:"
            <<min_param.a2<<" a3:"<<min_param.a3<<" a4: "<<min_param.a4<<" e1: "<<min_param.e1<<" e2:"<<min_param.e2<<" position:"
            <<min_param.pose.position.x<<" "<<min_param.pose.position.y<<min_param.pose.position.z<<" orientation:"
            <<min_param.pose.orientation.x<<" "<<min_param.pose.orientation.y<<" "<<min_param.pose.orientation.z<<" "
            <<min_param.pose.orientation.w<<std::endl;

  delete fit;

  Sampling *sam = new Sampling(min_param);
  //ALBA: use Pilu-Fisher instead
  sam->sample_pilu_fisher_st();
  pcl::PointCloud<PointT>::Ptr sampled_cloud(new pcl::PointCloud<PointT>);
  sam->getCloud(sampled_cloud);
  std::cout<<"Size of the sampled cloud: "<<sampled_cloud->points.size()<<std::endl;
  pcl::io::savePCDFileASCII ("/home/aperez/catkin_ws/STfittedOnly.pcd", *sampled_cloud);
  std::cerr << "Saved " << sampled_cloud->points.size () << " STfittedOnly.pcd." << std::endl;
  *sampled_cloud+=*cloud;
  pcl::io::savePCDFileASCII ("/home/aperez/catkin_ws/STfitted.pcd", *sampled_cloud);
  std::cerr << "Saved " << sampled_cloud->points.size () << " STfitted.pcd." << std::endl;



  ROS_INFO("Hello world!");
  return 0;
}

#include<iostream>
#include<supertoroid/st_sampling.h>
#include<supertoroid/st.h>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sampling_test_pcd_st");
  ros::NodeHandle nh("~");
  double a1_param;
  nh.getParam("/fittingST/a1param",a1_param);
  std::cout<<"a1 value: "<<a1_param<<std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined(new pcl::PointCloud<pcl::PointXYZRGB>());

  supertoroid::st super2;
  /* nh.getParam("/samplingST/a1param",super2.a1);*/
  nh.getParam("/stp/par_a1",super2.a1);
  nh.getParam("/stp/par_a2",super2.a2);
  nh.getParam("/stp/par_a3",super2.a3);
  nh.getParam("/stp/par_a4",super2.a4);
  nh.getParam("/stp/par_e1",super2.e1);
  nh.getParam("/stp/par_e2",super2.e2);
  geometry_msgs::Pose pose2;
  nh.getParam("/stp/par_x",pose2.position.x);
  nh.getParam("/stp/par_y",pose2.position.y);
  nh.getParam("/stp/par_z",pose2.position.z);
  //orientation in quaternions
/*   pose2.orientation.x = 0.0;
  pose2.orientation.y = sin(M_PI/8.);
  pose2.orientation.y = 0.0;
  pose2.orientation.z = 0.0;
  pose2.orientation.w = cos(M_PI/8.); 
  pose2.orientation.w = 1.0; */
  //Pass from axis-angle to quaternions
  double x_aa, y_aa, z_aa;
  nh.getParam("/stp/par_orx",x_aa);
  nh.getParam("/stp/par_ory",y_aa);  
  nh.getParam("/stp/par_orz",z_aa);
  //Transform to quaternion
  Eigen::Vector3d rot_aa(x_aa,y_aa,z_aa);
  auto ang_aa = rot_aa.norm();
  Eigen::Vector3d axis_aa = rot_aa.normalized();
  Eigen::Quaternion<double> myq;
  myq.x() = axis_aa.x()*sin(ang_aa/2.0);
  myq.y() = axis_aa.y()*sin(ang_aa/2.0);
  myq.z() = axis_aa.z()*sin(ang_aa/2.0);
  myq.w() = cos(ang_aa/2.0);
  pose2.orientation.x = myq.x();
  pose2.orientation.y = myq.y();
  pose2.orientation.z = myq.z();
  pose2.orientation.w = myq.w();
  super2.pose = pose2;
  std::cout<<"Parameters from yaml file: "<<super2.a1 <<"  "<<super2.a2<<"  "<<super2.a3
  <<"  "<<super2.a4<<"  "<<super2.e1<<"  "<<super2.e2<<"  "<<pose2.position.x<<"  "<<pose2.position.y
  <<"  "<<pose2.position.z<<"  "<<pose2.orientation.x<<"  "<<pose2.orientation.x<<"  "
  <<"  "<<pose2.orientation.z<<"  "<<pose2.orientation.w<<"  "<<std::endl;

  //Sampling *sam = new Sampling(super);

  //sam->sample_pilu_fisher_st();
  //pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  //sam->getCloud(cloud);
  //std::cout<<"size of the first sampled cloud: "<<cloud->points.size()<<std::endl;

  Sampling *sam2 = new Sampling(super2);
  sam2->sample_pilu_fisher_st();
  pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
  sam2->getCloud(cloud2);
  std::cout<<"size of the second sampled cloud: "<<cloud2->points.size()<<std::endl;

  *cloud_combined = *cloud2;
  //*cloud_combined+= *cloud2;

  std::cout<<"Size of combined superquadrics cloud: "<<cloud_combined->points.size()<<std::endl;
  std::string fileName = "/tmp/STsynth.pcd";
  pcl::io::savePCDFileASCII (fileName, *cloud_combined);
  std::cerr << "Saved " << cloud_combined->points.size () << " data points to " << fileName << std::endl;

  return 0;
}

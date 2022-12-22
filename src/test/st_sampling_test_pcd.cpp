#include<iostream>
#include<supertoroid/st_sampling.h>
#include<supertoroid/st.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

int main(int argc, char *argv[])
{
  pcl::PointCloud<PointT>::Ptr cloud_combined(new pcl::PointCloud<PointT>());

  supertoroid::st super1;
  super1.a1 = 0.1;
  super1.a2 = 0.1;
  super1.a3 = 0.1;
  super1.a4 = 0.0;
  super1.e1 = 1.0;
  super1.e2 = 1.0;
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  super1.pose = pose;

  /**
   * This is just to prove that both the sample and sample_new methods can provide the same results.
   * If we change the a1 or a2 in the sample_new() method, we have to calculate the a4 and then a1 and a2 accordingly
   * in the sample() method
   */


  supertoroid::st super2;
  super2.a1 = 0.01;
  super2.a2 = 0.01;
  super2.a3 = 0.1;
  super2.a4 = 0.15;
  super2.e1 = 1.0;
  super2.e2 = 1.0;
  geometry_msgs::Pose pose2;
  pose2.position.x = 0.3;
  pose2.position.y = 0.0;
  pose2.position.z = 0.0;
  pose2.orientation.w = 1.0;
  super2.pose = pose2;

  supertoroid::st super3;
  super3.a1 = 0.01;
  super3.a2 = 0.01;
  super3.a3 = 0.1;
  super3.a4 = 15;
  super3.e1 = 1.0;
  super3.e2 = 1.0;
  geometry_msgs::Pose pose3;
  pose3.position.x = -0.3;
  pose3.position.y = 0.0;
  pose3.position.z = 0.0;
  pose3.orientation.w = 1.0;
  super3.pose = pose3;

  /*Sampling *samp = new Sampling(super1);
  samp->sample_pilu_fisher();
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  samp->getCloud(cloud);
  std::cout<<"Size of the first sampled cloud: "<<cloud->points.size()<<std::endl;
  *cloud_combined+= *cloud;*/

  Sampling *samp2 = new Sampling(super2);
  samp2->sample_new();
  pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
  samp2->getCloud(cloud2);
  std::cout<<"Size of the second sampled cloud: "<<cloud2->points.size()<<std::endl;
  *cloud_combined+= *cloud2;

  Sampling *samp3 = new Sampling(super3);
  samp3->sample();
  pcl::PointCloud<PointT>::Ptr cloud3(new pcl::PointCloud<PointT>);
  samp3->getCloud(cloud3);
  std::cout<<"Size of the third sampled cloud: "<<cloud3->points.size()<<std::endl;
  *cloud_combined+= *cloud3;

  pcl::io::savePCDFileASCII ("supertoroid_test.pcd", *cloud_combined);
  std::cerr << "Saved " << cloud_combined->points.size () << " data points to supertoroid.pcd." << std::endl;
  return 0;

}

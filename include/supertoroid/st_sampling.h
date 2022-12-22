#ifndef ST_SAMPLING_H
#define ST_SAMPLING_H

#include<supertoroid/st.h>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointT;

class Sampling{
public:
  Sampling(const supertoroid::st& st_params);
  void sample();
  void sample_proper();
  void sample_new();
  void sample_pilu_fisher_st();

  void getCloud(pcl::PointCloud<PointT>::Ptr& cloud);
  void getCloud(sensor_msgs::PointCloud2& cloud_ros);

private:
  pcl::PointCloud<PointT>::Ptr cloud_;
  sensor_msgs::PointCloud2 cloud_ros_;
  supertoroid::st params_;
  float r_, g_, b_;
  void transformCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud,
                      pcl::PointCloud<PointT>::Ptr& output_cloud);

//  void sample_superEllipse_withOffset(const double a1, const double a2, const double a4, const double e, const int N, pcl::PointCloud<PointT>::Ptr &cloud);

};

#endif // ST_SAMPLING_H

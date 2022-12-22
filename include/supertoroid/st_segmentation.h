#ifndef ST_SEGMENTATION_H
#define ST_SEGMENTATION_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/surface/concave_hull.h>
#include<pcl/segmentation/extract_polygonal_prism_data.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr CloudPtr;

class Segmentation{
public:
  struct Parameters{
    double zmin;
    double zmax;
    };
  struct ws_Parameters{
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
  };

  Segmentation(const CloudPtr &input_cloud, const Parameters &param, const ws_Parameters& ws_param);
  bool segment();
  bool initialized;
  void getTablecloud(CloudPtr &table_cloud) const;
  void getObjectsOnTable(CloudPtr& objects_on_table) const;
  void getWsCloud(CloudPtr& ws_cloud) const ;

private:
  void detectObjectsOntable(CloudPtr cloud, double zmin, double zmax);


  CloudPtr cloud_;
  CloudPtr filtered_cloud_;
  CloudPtr ws_cloud_;
  CloudPtr table_plane_cloud_;
  CloudPtr objects_on_table_;
  pcl::ModelCoefficients plane_coefficients_;

  double zmin_;
  double zmax_;


  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;

};

#endif // ST_SEGMENTATION_H

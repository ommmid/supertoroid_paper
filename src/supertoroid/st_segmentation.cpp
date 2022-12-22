#include<supertoroid/st_segmentation.h>
#include<pcl/filters/crop_box.h>


Segmentation::Segmentation(const CloudPtr &input_cloud, const Parameters &param,const ws_Parameters& ws_param)
  :table_plane_cloud_(new PointCloud), objects_on_table_(new PointCloud), ws_cloud_(new PointCloud), filtered_cloud_
(new PointCloud){
  cloud_ = input_cloud;
  this->zmin_ = param.zmin;
  this->zmax_ = param.zmax;
  this->initialized = true;

  this->min_x_ = ws_param.min_x;
  this->max_x_ = ws_param.max_x;
  this->min_y_ = ws_param.min_y;
  this->max_y_ = ws_param.max_y;
  this->min_z_ = ws_param.min_z;
  this->max_z_ = ws_param.max_z;
}


void Segmentation::detectObjectsOntable(CloudPtr cloud, double zmin, double zmax){
  pcl::SACSegmentation<PointT> seg;
  CloudPtr cloud_nan(new PointCloud);
  CloudPtr convexHull(new PointCloud);
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(cloud);
  Eigen::Vector4f min;
  min<<min_x_, min_y_, min_z_,1;
  Eigen::Vector4f max;
  max<<max_x_, max_y_, max_z_,1;
  crop.setMin(min);
  crop.setMax(max);
  crop.filter(*cloud_nan);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_nan, *filtered_cloud_, indices);
  pcl::copyPointCloud(*filtered_cloud_, *ws_cloud_);
  seg.setInputCloud(filtered_cloud_);

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setOptimizeCoefficients(true);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  seg.segment(*planeIndices, this->plane_coefficients_);
  if(planeIndices->indices.size()==0)
    std::cout<<"Could not find a plane"<<std::endl;
  else{
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filtered_cloud_);
    extract.setIndices(planeIndices);
    extract.filter(*table_plane_cloud_);

    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(table_plane_cloud_);
    hull.setDimension(2);
    hull.reconstruct(*convexHull);
    pcl::PointIndices::Ptr obj_indices(new pcl::PointIndices());
    if(hull.getDimension() ==2){
      pcl::ExtractPolygonalPrismData<PointT> prism;
      prism.setInputCloud(filtered_cloud_);
      prism.setInputPlanarHull(convexHull);
      prism.setHeightLimits(this->zmin_, this->zmax_);
      prism.segment(*obj_indices);
      extract.setIndices(obj_indices);
      extract.filter(*filtered_cloud_);
      objects_on_table_ = filtered_cloud_;
    }
   }
}

void Segmentation::getTablecloud(CloudPtr &table_cloud) const {
  table_cloud = table_plane_cloud_;
}

void Segmentation::getObjectsOnTable(CloudPtr &objects_on_table) const{
  objects_on_table = objects_on_table_;
}

void Segmentation::getWsCloud(CloudPtr &ws_cloud) const {
  ws_cloud = ws_cloud_;
}

bool Segmentation::segment(){
  if(this->initialized){
    detectObjectsOntable(this->cloud_, this->zmin_, this->zmax_);
  }
}

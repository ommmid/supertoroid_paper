#include<supertoroid/st_fitting.h>
#include <unsupported/Eigen/NonLinearOptimization> //This is the Levenberg optimization
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <sys/time.h>

using namespace std::chrono;

Fitting::Fitting(const pcl::PointCloud<PointT>::Ptr &input_cloud) : pre_align_(true), pre_align_axis_(2){
  cloud_ = input_cloud;
}

void Fitting::getMinParams(supertoroid::st &param){
  param = params_;
}

void Fitting::getPreAlignedCloud(pcl::PointCloud<PointT>::Ptr &cloud){
  cloud = prealigned_cloud_;
}

void Fitting::setPreAlign(bool pre_align, int pre_align_axis){
  pre_align_ = pre_align;
  pre_align_axis_ = pre_align_axis;
}

//PCA for a first alignment of the clouds
void Fitting::preAlign(Eigen::Affine3f &transform, Eigen::Vector3f &variances){
  Eigen::Vector4f xyz_centroid;
  struct timespec tsi;
  struct timespec tsf;
  clock_gettime(CLOCK_REALTIME, &tsi);
  pcl::compute3DCentroid(*cloud_, xyz_centroid);
  clock_gettime(CLOCK_REALTIME, &tsf);
  int difft = tsf.tv_nsec - tsi.tv_nsec;
  std::cout << "Computing the centroid takes ";
  std::cout << difft << " ";
  Eigen::Affine3f transform_centroid = Eigen::Affine3f::Identity();
  transform_centroid.translation()<<-xyz_centroid(0), -xyz_centroid(1), -xyz_centroid(2);
  clock_gettime(CLOCK_REALTIME, &tsi);
  pcl::PCA<PointT> pca;
  pca.setInputCloud(cloud_);
  Eigen::Vector3f eigenValues = pca.getEigenValues();
  Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
  clock_gettime(CLOCK_REALTIME, &tsf);
  difft = tsf.tv_nsec - tsi.tv_nsec;
  std::cout << "Computing the PCA takes ";
  std::cout << difft << " ";
  double eig1,eig2,eig3;
  eig1 = eigenValues(0);
  eig2 = eigenValues(1);
  eig3 = eigenValues(2);
  Eigen::Vector3f vec_dir = eigenVectors.col(0);
  std::cout << "Eigenvectors ";
  std::cout << vec_dir(0) << " "<< vec_dir(1) << " "<< vec_dir(2) << " ";
  std::cout << "Eigenvalues ";
  std::cout << eig1<< " "<< eig2 << " "<< eig3 << " ";
  //Here we rotate the x,y,z axis according to who is x, keeping direct trihedron
  Eigen::Vector3f vec_aux = eigenVectors.col(0);
  eigenVectors.col(0) = eigenVectors.col(pre_align_axis_);
  eigenVectors.col(1) = eigenVectors.col(pre_align_axis_ + 1 - 3*floor((pre_align_axis_ + 1)/3));
  eigenVectors.col(2) = eigenVectors.col(pre_align_axis_ + 2 - 3*floor((pre_align_axis_ + 2)/3));
  //eigenVectors.col(pre_align_axis_) = vec_aux;
  float aux_ev = eigenValues(0);
  eigenValues(0) = eigenValues(pre_align_axis_);
  eigenValues(1) = eigenValues(pre_align_axis_ + 1 - 3*floor((pre_align_axis_ + 1)/3));
  eigenValues(2) = eigenValues(pre_align_axis_ + 2 - 3*floor((pre_align_axis_ + 2)/3));
  //eigenValues(pre_align_axis_) = aux_ev;
  Eigen::Matrix4f transformation_pca = Eigen::Matrix4f::Identity();
  for(int i=0;i<transformation_pca.cols()-1;++i){
    for(int j=0;j<transformation_pca.rows()-1;++j){
      transformation_pca(j,i) = eigenVectors(i,j);
    }
  }
  Eigen::Affine3f transformation_pca_affine;
  transformation_pca_affine.matrix() = transformation_pca;
  transform = transformation_pca_affine * transform_centroid;

  eigenValues /= static_cast<float>(cloud_->size());
  variances(0) = sqrt(eigenValues(0));
  variances(1) = sqrt(eigenValues(1));
  variances(2) = sqrt(eigenValues(2));
}

//Parameters (intrinsic+extrinsic) in xvec.
//Transform all points of the cloud to orient it according to trans. Values of trans obtained from PCA
//The aligned cloud 
int Fitting::OptimizationFunctor::operator ()(const Eigen::VectorXd &xvec, Eigen::VectorXd &fvec) const{
  pcl::PointCloud<PointT>::Ptr cloud_new(new pcl::PointCloud<PointT>);
  cloud_new = estimator_->prealigned_cloud_;
  double a = xvec[0], b = xvec[1], c = xvec[2], d = xvec[3], e1 = xvec[4], e2 = xvec[5];
  Eigen::Affine3d trans;
  create_transformation_matrix(xvec[6], xvec[7], xvec[8], xvec[9], xvec[10], xvec[11], trans);
  pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud_new, *cloud_transformed, trans);
  for(int i=0;i<values();++i)
  {
    Eigen::Matrix<double , 4, 1>xyz_tr(cloud_transformed->at(i).x, cloud_transformed->at(i).y, cloud_transformed->at(i).z, 1.);
    double op = Eigen::Matrix<double, 3, 1>(xyz_tr[0], xyz_tr[1], xyz_tr[2]).norm();
    PointT p;
    p.x = xyz_tr[0];
    p.y = xyz_tr[1];
    p.z = xyz_tr[2];
    //ALBA: this is the wrong distance function
    //fvec[i] = op * st_function(xyz_tr[0], xyz_tr[1], xyz_tr[2], a,b,c,d,e1,e2);
    fvec[i] = (1-st_function_b1(xyz_tr[0], xyz_tr[1], xyz_tr[2], a,b,c,d,e1,e2))
        *sqrt((1-st_function_b2(xyz_tr[0], xyz_tr[1], a, b, c, d, e1, e2))*(1-st_function_b2(xyz_tr[0], xyz_tr[1], a, b, c, d, e1, e2))
        *((p.x*p.x)+(p.y*p.y))+(p.z*p.z));
  }
  return (0);
}

void Fitting::fit(){
  double min_fit_error = std::numeric_limits<double>::max();
  supertoroid::st min_param;
  //ALBA: modification just for debugging, set back to 3 later if needed (it rotates through x,y,z to guess z)
  for(int i=0;i<3;++i)
  {
    double error;
    setPreAlign(true, i);
    supertoroid::st param;
    fit_param(param, error);
    std::cout<<"Error is: "<<error<<std::endl;
    std::cout<<"Min param is: "<<param.a1<<" "<<param.a2<<" "<<param.a3<<" "<<param.a4<<std::endl;
    std::cout<<param.e1<<" "<<param.e2<<std::endl;
    if(error<min_fit_error){
      min_fit_error = error;
      min_param = param;
    }
  }
  params_ = min_param;
}

void Fitting::fit_param(supertoroid::st &param, double &final_error)
{
  Eigen::Affine3f transform_inv;
  Eigen::Vector3f variances;
  struct timespec tsi;
  struct timespec tsf;
  if(pre_align_){
    clock_gettime(CLOCK_REALTIME, &tsi);
    preAlign(transform_inv, variances);
    prealigned_cloud_.reset(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud_,*prealigned_cloud_, transform_inv);
    clock_gettime(CLOCK_REALTIME, &tsf);
    int difft = tsf.tv_nsec - tsi.tv_nsec;
    std::cout << "Transforming the cloud Using PCA takes ";
    std::cout << difft << " ";
    //ALBA: Here I am trying to get some of the transformed points of the cloud
    //pcl::PointXYZRGB minpt, maxpt;
    //pcl::getMinMax3D (*prealigned_cloud_,minpt, maxpt);
    //std::cout << "min max points results";
    //std::cout << minpt.x << " " << minpt.y << " " << minpt.z;
    //std::cout << maxpt.x << " " << maxpt.y << " " << maxpt.z;
    //ALBA up to here
  }


  Eigen::Affine3d trans_new = (transform_inv.inverse().cast<double>());
  double tx, ty, tz, ax, ay, az;
  getParamFromTransform(trans_new, tx, ty, tz, ax, ay, az);

  int n_unknowns = 12;
  Eigen::VectorXd xvec(n_unknowns);
  //CHANGES ALBA
  //Initial conditions Abhijit
  //xvec[0] = variances(0) * 3.;
  //xvec[1] = variances(1) * 3.;
  //xvec[2] = variances(2) * 3.;
  //xvec[3] = variances(0) * 300.;
  //Initial conditions closer to the torus geometry
  //They are scaled by a1 = 1, if this is far, everything will be
  //xvec[0] = 1.0;
  //xvec[1] = variances(1)/variances(0)*xvec[0];
  //xvec[2] = variances(2);
  //xvec[3] = variances(0)/xvec[0];
  //xvec[4] = 1.0;
  //xvec[5] = 1.0;
  std::cout << "Values of t for checking: " << tx << " " << ty << " " << tz;
  std::cout << "Values of Euler angles for checking: " << ax << " " << ay << " " << az;
  xvec[6] = tx;
  xvec[7] = ty; 
  xvec[8] = tz;
  xvec[9] = xvec[10] =  xvec[11] =0.;
  //inital conditions to match the sampled supertoroid, for testing:
  xvec[0] = 6.0;
  xvec[1] = 2.0;
  xvec[2] = 5.0;
  xvec[3] = 4.0;
  xvec[4] = 0.3;
  xvec[5] = 1.4;  
  //Try to read params from launch file:
  //0-5 intrinsic, 6-8 location, 9-11 axis-angle orientation (dir vector*angle)
  ros::NodeHandle nh("~");
  nh.getParam("/stp/par_a10",xvec[0]);
  nh.getParam("/stp/par_a20",xvec[1]);
  nh.getParam("/stp/par_a30",xvec[2]);
  nh.getParam("/stp/par_a40",xvec[3]);
  nh.getParam("/stp/par_e10",xvec[4]);
  nh.getParam("/stp/par_e20",xvec[5]);
  nh.getParam("/stp/par_x",xvec[6]);
  nh.getParam("/stp/par_y",xvec[7]);
  nh.getParam("/stp/par_z",xvec[8]);
  nh.getParam("/stp/par_orx",xvec[9]);
  nh.getParam("/stp/par_ory",xvec[10]);
  nh.getParam("/stp/par_orz",xvec[11]);
  std::cout << " My initial a1 to a4 LAUNCH FILE";
  std::cout << xvec[0] << " " << xvec[1] << " " << xvec[2] << " " << xvec[3];

  OptimizationFunctor functor(prealigned_cloud_->size(), this);
  Eigen::NumericalDiff<OptimizationFunctor> numericalDiffMyFunctor(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm(numericalDiffMyFunctor);
  lm.parameters.maxfev = 20000;
  lm.parameters.xtol = 1.0e-12;
  lm.parameters.ftol = 1.0e-12;
  std::cout << lm.parameters.maxfev << std::endl;
  auto start = high_resolution_clock::now();
  int ret = lm.minimize(xvec);
  auto stop = high_resolution_clock::now();
  std::cout << "Iterations " << lm.iter << std::endl;
  std::cout << "Termination flag?" << ret << std::endl;
  std::cout << "Time in ms " << duration_cast<milliseconds>(stop - start).count() << std::endl;

  param.a1 = xvec[0];
  param.a2 = xvec[1];
  param.a3 = xvec[2];
  param.a4 = xvec[3];
  double e1 = xvec[4];
  double e2 = xvec[5];
  double e1nc = xvec[4];
  double e2nc = xvec[5];
  st_clampParameters(e1, e2);
  std::cout<<"THIS IS AFTER MINIMIZATION! \n";
  std::cout<<"a1 is: "<<param.a1<<std::endl;
  std::cout<<"a2 is: "<<param.a2<<std::endl;
  std::cout<<"a3 is: "<<param.a3<<std::endl;
  std::cout<<"a4 is: "<<param.a4<<std::endl;
  std::cout<<"E1nc is: "<<e1nc<<std::endl;
  std::cout<<"E2nc is: "<<e2nc<<std::endl;
  param.e1 = e1;
  param.e2 = e2;
  std::cout<<"E1 is: "<<param.e1<<std::endl;
  std::cout<<"E2 is: "<<param.e2<<std::endl;
  Eigen::Affine3d transform_lm;
  create_transformation_matrix(xvec[6], xvec[7], xvec[8], xvec[9], xvec[10], xvec[11], transform_lm);

  Eigen::Affine3f transform = transform_inv.inverse();
  Eigen::Affine3d final_transform = transform.cast<double>() * transform_lm ;
  Eigen::Vector3d t = final_transform.translation();
  param.pose.position.x = t(0);
  param.pose.position.y = t(1);
  param.pose.position.z = t(2);
  Eigen::Matrix3d rot_matrix = final_transform.rotation();
  Eigen::Quaterniond q(rot_matrix);
  q.normalize();
  param.pose.orientation.x = q.x();
  param.pose.orientation.y = q.y();
  param.pose.orientation.z = q.z();
  param.pose.orientation.w = q.w();

  //Creating a new param with pose of transform_lm to calculate error
  supertoroid::st param_lm;
  param_lm.a1 = param.a1;
  param_lm.a2 = param.a2;
  param_lm.a3 = param.a3;
  param_lm.a4 = param.a4;
  param_lm.e1 = param.e1;
  param_lm.e2 = param.e2;
  Eigen::Vector3d t1 = transform_lm.translation();
  param_lm.pose.position.x = t1(0);
  param_lm.pose.position.y = t1(1);
  param_lm.pose.position.z = t1(2);
  Eigen::Matrix3d rot_matrix_lm = transform_lm.rotation();
  Eigen::Quaterniond q1(rot_matrix_lm);
  q1.normalize();
  param_lm.pose.orientation.x = q1.x();
  param_lm.pose.orientation.y = q1.y();
  param_lm.pose.orientation.z = q1.z();
  param_lm.pose.orientation.w = q1.w();
  final_error = st_error(cloud_, param_lm);

}

#ifndef ST_FITTING_H
#define ST_FITTING_H
#include<iostream>
#include<supertoroid/st.h>
#include<supertoroid/util.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/common/centroid.h>
#include<pcl/common/pca.h>

typedef pcl::PointXYZRGB PointT;

class Fitting{
public:
  Fitting(const pcl::PointCloud<PointT>::Ptr& input_cloud);
  Fitting& operator = (const Fitting& src){}

  void setPreAlign(bool pre_align, int pre_align_axis = 2);
  void preAlign(Eigen::Affine3f& transform, Eigen::Vector3f& variances);
  void fit_param(supertoroid::st& param, double& final_error);
  void fit();
  void getPreAlignedCloud(pcl::PointCloud<PointT>::Ptr& cloud);
  void getMinParams(supertoroid::st& param);
  void getMinError(double& error);

private:
  pcl::PointCloud<PointT>::Ptr cloud_;
  pcl::PointCloud<PointT>::Ptr prealigned_cloud_;
  supertoroid::st params_;
  bool pre_align_;
  int pre_align_axis_;
  double min_error_;

  template<typename _Scalar, int nX = Eigen::Dynamic, int nY = Eigen::Dynamic>
  struct Functor
  {
    typedef _Scalar Scalar;
    enum
    {
      InputsAtCompileTime = nX,
      ValuesAtCompileTime = nY
    };
    typedef Eigen::Matrix<_Scalar, InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<_Scalar, ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<_Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    Functor() : m_data_points_(ValuesAtCompileTime){}
    Functor(int m_data_points) : m_data_points_(m_data_points){}
    virtual ~Functor(){}
    int values() const {return (m_data_points_);}
    int inputs() const {return 12;}
  protected:
    int m_data_points_;
  };

  struct OptimizationFunctor : public Functor<double>
  {
    using Functor<double>::values;
    OptimizationFunctor(int m_data_points, Fitting *estimator):
      Functor<double> (m_data_points), estimator_(estimator){}

    inline OptimizationFunctor(const OptimizationFunctor *src):
      Functor<double>(src->m_data_points_),estimator_()
    {
      *this = src;
    }

    virtual ~OptimizationFunctor(){}
    inline OptimizationFunctor& operator = (const OptimizationFunctor &src)
    {
      Functor<double>::operator = (src);
      estimator_ = src.estimator_;
      return (*this);
    }
  
    int operator () (const Eigen::VectorXd &xvec, Eigen::VectorXd &fvec) const;
    int df(const Eigen::VectorXd &xvec, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &fjac) const;

    Fitting* estimator_;
  };
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



#endif // ST_FITTING_H

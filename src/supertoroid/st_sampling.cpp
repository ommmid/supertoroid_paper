#include<supertoroid/st_sampling.h>
#include<ctime>
#include<supertoroid/util.h>

Sampling::Sampling(const supertoroid::st &st_params):
  params_(st_params), cloud_(new pcl::PointCloud<PointT>){

  struct timeval time;
  gettimeofday(&time, NULL);
  srand((time.tv_sec * 1000)+(time.tv_usec/1000));
  r_ = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
  g_ = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
  b_ = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
}


void Sampling::transformCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud,
                              pcl::PointCloud<PointT>::Ptr &output_cloud){
  Eigen::Affine3f transform;
  pose_to_transform(params_.pose, transform);
  pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
}


void Sampling::getCloud(pcl::PointCloud<PointT>::Ptr &cloud)
{
  cloud = cloud_;
}



double signof(double a) { return (a == 0) ? 0 : (a<0 ? -1 : 1); }

void Sampling::sample_new()
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    double cn, sn, sw, cw;
    double n,w;
    int num_n, num_w;
    double dn, dw;
    dn =  M_PI/180.0;
    dw =  M_PI/180.0;
    num_n = (int)(2.0*(M_PI/dn));
    num_w = (int)(2.0*(M_PI/dw));
    n = 0.0;
    for(int i=0;i<num_n;++i)
    {
      n+=dn;
      cn = cos(n);
      sn = sin(n);
      w = 0.0;
      for(int j=0;j<num_w;++j)
      {
        w+=dw;
        cw = cos(w);
        sw = sin(w);
        PointT p;

        p.x =  (params_.a4+params_.a1 *signof(cn)*pow(fabs(cn), params_.e1)) * pow (fabs(cw), params_.e2);
        p.y =  (params_.a4+params_.a2 *signof(cn)*pow(fabs(cn), params_.e1)) * pow (fabs(sw), params_.e2);
        p.z = params_.a3 * pow(fabs(sn), params_.e1);
        p.r =  r_*255;
        p.g = g_*255;
        p.b = b_*255;

        if(cn*cw <0){p.x = -p.x;}
        if(cn*sw <0){p.y = -p.y;}
        if(sn<0){p.z = -p.z;}
        cloud->points.push_back(p);
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = true;
      }
    }
    transformCloud(cloud, cloud_);
}


void Sampling::sample()
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    double cn, sn, sw, cw;
    double n,w;
    int num_n, num_w;
    double dn, dw;
    dn =  M_PI/180.0;
    dw =  M_PI/180.0;
    num_n = (int)(2.0*(M_PI/dn));
    num_w = (int)(2.0*(M_PI/dw));
    n = 0.0;
    for(int i=0;i<num_n;++i)
    {
      n+=dn;
      cn = cos(n);
      sn = sin(n);
      w = 0.0;
      for(int j=0;j<num_w;++j)
      {
        w+=dw;
        cw = cos(w);
        sw = sin(w);
        PointT p;

        p.x = params_.a1 * (params_.a4+signof(cn)*pow(fabs(cn), params_.e1)) * pow (fabs(cw), params_.e2);
        p.y = params_.a2 * (params_.a4+signof(cn)*pow(fabs(cn), params_.e1)) * pow (fabs(sw), params_.e2);
        p.z = params_.a3 * pow(fabs(sn), params_.e1);
        p.r = r_*255;
        p.g = g_*255;
        p.b = b_*255;

        if(cn*cw <0){p.x = -p.x;}
        if(cn*sw <0){p.y = -p.y;}
        if(sn<0){p.z = -p.z;}
        cloud->points.push_back(p);
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = true;
      }
    }
    transformCloud(cloud, cloud_);
}


void Sampling::sample_proper()
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    double cn, sn, sw, cw;
    double n,w;
    int num_n, num_w;
    double dn, dw;
    dn =  M_PI/180.0;
    dw =  M_PI/180.0;
    num_n = (int)(2.0*(M_PI/dn));
    num_w = (int)(2.0*(M_PI/dw));
    n = 0.0;
    for(int i=0;i<num_n;++i)
    {
      n+=dn;
      cn = cos(n);
      sn = sin(n);
      w = 0.0;
      for(int j=0;j<num_w;++j)
      {
        w+=dw;
        cw = cos(w);
        sw = sin(w);
        PointT p;

        p.x = params_.a1 * (params_.a4+pow(cn, params_.e1)) * pow (fabs(cw), params_.e2);
        p.y = params_.a2 * (params_.a4+pow(cn, params_.e1)) * pow (fabs(sw), params_.e2);
        p.z = params_.a3 * pow(fabs(sn), params_.e1);
        p.r =  r_*255;
        p.g = g_*255;
        p.b = b_*255;

        if(cn*cw <0){p.x = -p.x;}
        if(cn*sw <0){p.y = -p.y;}
        if(sn<0){p.z = -p.z;}
        cloud->points.push_back(p);
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = true;
      }
    }
    transformCloud(cloud, cloud_);
}

   // ------------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------- supertroid ---------------------------------------------------------------------------------------
  // paper: Equal-Distance Sampling of Superellipse Models by Fisher 1995
  // In functions: dTheta_0, dTheta, eq(5) which is basically the derivative is used. So we do not need to be worried about parameter "a"
  // as it is a constant value in supertroid. So both of these functions are usable for supertptoids without any change
static double dTheta_0(double K, double e, double a1, double a2, double t)
{
  double factor = K/a2 - pow(t,e);
  double po = pow(fabs(factor), 1.0/e);
  double m = fabs(po - t);
  return m;
}

static double dTheta(double K, double e, double a1, double a2, double t)
{
  double num = (cos(t)* cos(t)* sin(t)*sin(t));
  double den1 = a1*a1*pow(fabs(cos(t)), 2*e) * pow(fabs(sin(t)),4);
  double den2 = a2*a2*pow(fabs(sin(t)), 2*e) * pow(fabs(cos(t)),4);
  return (K/e)*sqrt(num/(den1+den2));
}
// supertoroid is the spherical product of a superellipse_with_offset and another superellipse
// here I change sample_superEllipse function to have offset
static void sample_superEllipse_withOffset(const double a1, const double a2, const double a4, const double e, const int N, pcl::PointCloud<PointT>::Ptr &cloud)
{
  // a is the offset
  cloud->points.resize(0);
  pcl::PointCloud<PointT>::Ptr cloud_base(new pcl::PointCloud<PointT>);
  double theta;
  double thresh = 0.1;
  int numIter;
  int maxIter = 500;
  double K;
  if(a1<a2){K = 2*M_PI*a1/(double)N;}
  else{K = 2*M_PI*a2/(double)N;}

  int signsX[4] = {1, -1,  1, -1};
  int signsY[4] = {1, -1, -1,  1};

  // for  theta close to 0
  theta = 0;
  numIter = 0;
  do
  {
    double dt = dTheta_0(K, e, a1, a2, theta);
    theta +=dt;
    numIter++;

    if(dt !=0)
    {
      PointT p;
      double  tempx = pow(fabs(cos(theta)), e);
      double  tempy = pow(fabs(sin(theta)), e);
      for(int z = 0; z < 4; ++z){
        p.x = a4 + a1 * signsX[z] * tempx;
        p.y = a2  * signsY[z] * tempy;
        p.z = 0;
        cloud_base->points.push_back(p);
      }
    }
  }while(theta<thresh && numIter<maxIter);

 // for theta greater than 0 and less than pi/2
  if(theta<thresh){theta = thresh;}
  numIter = 0;
  do
  {
    theta +=dTheta(K, e, a1,a2, theta);
    numIter++;
    PointT p;
    double  tempx =  pow(fabs(cos(theta)), e);
    double  tempy = pow(fabs(sin(theta)), e);
    for(int z = 0; z < 4; ++z){
      p.x = a4 + a1 * signsX[z] * tempx;
      p.y = a2  * signsY[z] * tempy;
      p.z = 0;
      cloud_base->points.push_back(p);
    }
  }
  while(theta<M_PI/2.0 - thresh && numIter<maxIter);


  // for theta close to pi/2
  double alpha = M_PI/2.0 - theta;
  numIter = 0;
  while(alpha>0 && numIter<maxIter)
  {
    alpha -=dTheta(K, e, a2, a1, alpha);
    numIter++;
    PointT p;
   double  tempx = pow(fabs(sin(alpha)),e);
    double  tempy = pow(fabs(cos(alpha)),e);
    for(int z = 0; z < 4; ++z){
      p.x = a4 + a1 * signsX[z] * tempx;
      p.y = a2  * signsY[z] * tempy;
      p.z = 0;
      cloud_base->points.push_back(p);
    }
  }

    *cloud = *cloud_base;
    cloud->width = 1;
    cloud->height = cloud->points.size();
  }

  // sampling for superetroid based on Pilu_Fisher method
  void Sampling::sample_pilu_fisher_st()
  {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_s1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_s2(new pcl::PointCloud<PointT>);
    int N = 300;
    // supetroid is the spherical product of a superellipse_with_offset and another superellipse
    // [a+1*cos(v)^e1, a3*sin(v)^e1] o [a1*cos(w)^e2, a2*sin(w)^e2]
    sample_superEllipse_withOffset(1, params_.a3, params_.a4, params_.e1, N, cloud_s1);
    sample_superEllipse_withOffset(params_.a1, params_.a2, 0, params_.e2, N, cloud_s2);

    PointT p1, p2;
    int n = cloud_s1->points.size();
    for(int i=0; i<n; ++i)
    {
      p1 = cloud_s1->points[i];
      for(int j=0;j<cloud_s2->points.size();++j)
      {
        p2 = cloud_s2->points[j];
        PointT p;
        p.x = p1.x * p2.x;
        p.y = p1.x * p2.y;
        p.z = p1.y;
        p.r =  r_*255;
        p.g = g_*255;
        p.b = b_*255;
        cloud->points.push_back(p);
      }
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = true;
    transformCloud(cloud, cloud_);
  }





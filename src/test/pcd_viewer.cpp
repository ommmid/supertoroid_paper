#include <ros/ros.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <supertoroid/st_sampling.h>

//http://www.pointclouds.org/documentation/tutorials/cloud_viewer.php#cloud-viewer

void display_pcd(std::string pcdFile){ // name of the file: str

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::io::loadPCDFile (pcdFile, *cloud);

    pcl::visualization::CloudViewer viewer ("Alba Cloud Viewer"); // this input is goint to be the title of the window of the viewer
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcd_viewer");
  ros::NodeHandle nh;

  //  std::cout << "enter the name of the file: " << std::endl;
  //std::string fileName;
  // cin >> fileName;
 //fileName = "supt.pcd";
  std::string fileName = argv[1];
  std::cout << "reading file: " << fileName << std::endl;
  display_pcd(fileName);

}

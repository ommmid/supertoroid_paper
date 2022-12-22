#include<pcl/io/openni_grabber.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/console/parse.h>
#include<iostream>

using namespace std;
using namespace pcl;

PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>);
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);
boost::shared_ptr<visualization::CloudViewer> viewer;
Grabber* openniGrabber;
unsigned int filesSaved = 0;
bool saveCloud(false), noColor(false);

void printUsage(const char* programName)
{
    cout << "Usage: " << programName << " [options]"
                     << endl
                     << endl
                     << "Options:\n"
                     << endl
                     << "\t<none>     start capturing from an OpenNI device.\n"
                     << "\t-v FILE    visualize the given .pcd file.\n"
                     << "\t-h         shows this help.\n";
}

void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
    if(!viewer->wasStopped())
        viewer->showCloud(cloud);
    if(saveCloud)
    {
        stringstream stream;
        stream<<"inputCloud"<<filesSaved<<".pcd";
        string filename = stream.str();
        if(io::savePCDFileASCII(filename, *cloud)== 0)
        {
            filesSaved++;
            cout<<"Saved "<<filename<<"."<<endl;
        }
        else PCL_ERROR("Problem saveing %s.\n",filename.c_str());
        saveCloud = false;
    }
}

void keyboardEventOccured(const visualization::KeyboardEvent& event, void* nothing)
{
    if(event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
}

boost::shared_ptr<visualization::CloudViewer> createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("Openni viewer"));
    v->registerKeyboardCallback(keyboardEventOccured);
    return v;
}

int main(int argc, char** argv)
{
    if(console::find_argument(argc, argv, "-h")>=0)
    {
        printUsage(argv[0]);
        return -1;
    }
    bool justVisualize(false);
    string filename;
    if(console::find_argument(argc, argv,"-v")>=0)
    {
        if(argc!=3)
        {
            printUsage(argv[0]);
            return -1;
        }
        filename = argv[2];
        justVisualize = true;
    }
    else if(argc!=1)
    {
        printUsage(argv[0]);
        return -1;
    }

    if(justVisualize)
    {
        try{
            io::loadPCDFile<PointXYZRGBA>(filename.c_str(), *cloudptr);
        }
        catch(PCLException e1)
        {
            try{
                io::loadPCDFile<PointXYZ>(filename.c_str(), *fallbackCloud);
            }
            catch(PCLException e2)
            {
                return -1;
            }
            noColor = true;
        }
        cout<<"Loaded "<<filename<<"."<<endl;
        if(noColor)
            cout<<"This cloud does not hve any color info."<<endl;
        else cout<<"This cloud has color info"<<endl;
    }

    else{
        openniGrabber = new OpenNIGrabber();
        if(openniGrabber == 0)
            return -1;
        boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
                boost::bind(&grabberCallback, _1);
        openniGrabber->registerCallback(f);
    }
    viewer = createViewer();
    if(justVisualize)
    {
        if(noColor)
            viewer->showCloud(fallbackCloud);
        else viewer->showCloud(cloudptr);
    }
    else openniGrabber->start();

    while(!viewer->wasStopped())
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    if(!justVisualize)
        openniGrabber->stop();

}

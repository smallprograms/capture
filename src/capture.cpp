#include <sstream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

using namespace std;
            
        

class SimpleOpenNIProcessor
{
    public:
        SimpleOpenNIProcessor(pcl::visualization::CloudViewer& viewer);
        void memberFunction (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in);
    private:
        pcl::visualization::CloudViewer* viewer;
};

SimpleOpenNIProcessor::SimpleOpenNIProcessor(pcl::visualization::CloudViewer& viewer) {
    this->viewer = &viewer;
}

void SimpleOpenNIProcessor::memberFunction(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in)
{
    std::stringstream ss;
    static int count=0;
    pcl::PointCloud<pcl::PointXYZ> cloud(640,480);
    std::vector<int> vec;
    //filter 
    pcl::removeNaNFromPointCloud( *cloud_in, cloud, vec );
    pcl::PointCloud<pcl::PointXYZ>::Ptr fCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud.makeShared());
    filter.setFilterFieldName("z");
    //0 to 1 m
    filter.setFilterLimits(0,1);
    filter.filter(*fCloud);

    if( fCloud->size() > 10) {
        //show and save 
        viewer->showCloud(fCloud);
        count++;
        std::cout << "Frame : " << count << "\n";
        ss << "pcd/cap" << count << ".pcd";
        pcl::io::savePCDFile(ss.str(),*fCloud,true);
    }    
}



int main(int argc, char** argv)
{
    //window to render
    pcl::visualization::CloudViewer viewer("Dots");

    //comunication with OpenNI module 
    pcl::OpenNIGrabber* grabber = new pcl::OpenNIGrabber();

    //custom class
    SimpleOpenNIProcessor processor(viewer);

    //connect some method of our custom class with the OpenNI grabber
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> member_function =
        boost::bind (&SimpleOpenNIProcessor::memberFunction, &processor, _1);
    grabber->registerCallback(member_function);

    grabber->start();
    while(true);
    grabber->stop ();

    return 0;
}


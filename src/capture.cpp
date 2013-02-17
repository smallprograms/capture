#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

using namespace std;
            

class SimpleOpenNIProcessor
{
    public:
        SimpleOpenNIProcessor(pcl::visualization::CloudViewer& viewer);
        void cbPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud_in);
        void cbImage(const boost::shared_ptr<openni_wrapper::Image>& im);
        cv::Mat getFrame (const boost::shared_ptr<openni_wrapper::Image> &img);
    private:
        pcl::visualization::CloudViewer* viewer;
};

SimpleOpenNIProcessor::SimpleOpenNIProcessor(pcl::visualization::CloudViewer& viewer) {
    this->viewer = &viewer;
}

void SimpleOpenNIProcessor::cbPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud_in)
{
    std::stringstream ss;
    static int count=0;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud(640,480);
    std::vector<int> vec;
    //filter 
    pcl::removeNaNFromPointCloud( *cloud_in, cloud, vec );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr fCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PassThrough<pcl::PointXYZRGBA> filter;
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

void SimpleOpenNIProcessor::cbImage(const boost::shared_ptr<openni_wrapper::Image>& im) {
    //cv::imwrite("adf.jpg",getFrame(im));
}

cv::Mat SimpleOpenNIProcessor::getFrame (const boost::shared_ptr<openni_wrapper::Image> &img) {

  cv::Mat frameRGB=cv::Mat(img->getHeight(),img->getWidth(),CV_8UC3);

  img->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
  cv::Mat frameBGR;
  cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);

  return frameBGR;

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
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> pcbPointCloud =
        boost::bind (&SimpleOpenNIProcessor::cbPointCloud, &processor, _1);

    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> pcbImage =
        boost::bind (&SimpleOpenNIProcessor::cbImage, &processor, _1);
    grabber->registerCallback(pcbPointCloud);
    grabber->registerCallback(pcbImage);

    grabber->start();
    while(true);
    grabber->stop ();

    return 0;
}


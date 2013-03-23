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
#include <QtConcurrentRun>
#include <QMutexLocker>
#include <QMutex>
#include <queue>

using namespace std;

bool quit=false;
/** capture keyboard keyDown event **/
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
        void *not_used)
{

    if (event.getKeyCode() == 27 && event.keyDown ())
    {
        quit = true;
    }
}


class SimpleOpenNIProcessor  : public QThread
{
    public:
        SimpleOpenNIProcessor(pcl::visualization::CloudViewer& viewer);
        void cbPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud_in);
        void saveCloudToFile(const std::string name, const pcl::PointCloud<pcl::PointXYZRGBA> cloud);
        void saveImageToFile(const std::string name,const cv::Mat matIn);
        std::vector< QFuture<void> > threadVec;
        void cbImage(const boost::shared_ptr<openni_wrapper::Image>& im);
        cv::Mat getFrame (const boost::shared_ptr<openni_wrapper::Image> &img);
        ~SimpleOpenNIProcessor();
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
//    std::vector<int> vec;
//    //filter
//    pcl::removeNaNFromPointCloud( *cloud_in, cloud, vec );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr fCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PassThrough<pcl::PointXYZRGBA> filter;
    //we want to preserve cloud org. and just use NaN to indicate non valid points
    filter.setKeepOrganized(true);
    filter.setInputCloud(cloud_in);
    filter.setFilterFieldName("z");
    //0 to 3 m
    filter.setFilterLimits(0,3);
    filter.filter(*fCloud);
    const int MIN_POINTS = 5000;
    if( fCloud->size() > MIN_POINTS) {
        //show and save 
        viewer->showCloud(fCloud);
        count++;
        std::cout << "Frame : " << count << "\n";
        ss << "pcd/cap" << count << ".pcd";
        threadVec.push_back( QtConcurrent::run(this, &SimpleOpenNIProcessor::saveCloudToFile,ss.str(),*fCloud));
    }
}

void SimpleOpenNIProcessor::saveCloudToFile(const std::string name,const pcl::PointCloud<pcl::PointXYZRGBA> cloud)
{
    pcl::io::savePCDFile(name,cloud,true);
}

void SimpleOpenNIProcessor::saveImageToFile(const std::string name, const cv::Mat matIn)
{
    cv::imwrite(name,matIn);
}

void SimpleOpenNIProcessor::cbImage(const boost::shared_ptr<openni_wrapper::Image>& im) {
//    static int j=0;
//    std::stringstream name;
//    name << "pcd/cap" << j << ".jpg";
//    threadVec.push_back( QtConcurrent::run(this,&SimpleOpenNIProcessor::saveImageToFile, name.str(), getFrame(im)));
//    j++;
}

cv::Mat SimpleOpenNIProcessor::getFrame (const boost::shared_ptr<openni_wrapper::Image> &img) {

  cv::Mat frameRGB=cv::Mat(img->getHeight(),img->getWidth(),CV_8UC3);

  img->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
  cv::Mat frameBGR;
  cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);

  return frameBGR;

}

SimpleOpenNIProcessor::~SimpleOpenNIProcessor()
{
    for(int j=0; j < threadVec.size(); j++) {
        threadVec[j].waitForFinished();
    }
}

int main(int argc, char** argv)
{
    //window to render
    pcl::visualization::CloudViewer viewer("Dots");
    viewer.registerKeyboardCallback( keyboardEventOccurred );

    //comunication with OpenNI module 
    pcl::OpenNIGrabber* grabber = new pcl::OpenNIGrabber();

    //custom class
    SimpleOpenNIProcessor processor(viewer);

    //connect some method of our custom class with the OpenNI grabber
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> pcbPointCloud =
        boost::bind (&SimpleOpenNIProcessor::cbPointCloud, &processor, _1);

//    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> pcbImage =
//        boost::bind (&SimpleOpenNIProcessor::cbImage, &processor, _1);
    grabber->registerCallback(pcbPointCloud);
//    grabber->registerCallback(pcbImage);

    grabber->start();
    while(true && !quit);
    grabber->stop ();

    return 0;
}


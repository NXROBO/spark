#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>

static const std::string MATCH_WINDOW = "match window";
static const std::string TEMPLATE_WINDOW = "template window";
static const std::string SUBSCRIBLE_TOPIC="/camera/rgb/image_rect_color";
//static const std::string SUBSCRIBLE_TOPIC="/usb_cam/image_raw";
static const double THRESHOLD = 7.5e+6;

class ImageProcessor
{
public:
    ImageProcessor(){}
    ~ImageProcessor(){}

    void process(cv::Mat _img)
    {
        cv::Mat gray_img ,edges,result;
        cv::cvtColor(_img,gray_img,CV_BGR2GRAY);
        cv::Canny(gray_img,edges,30,90);
        cv::matchTemplate(edges,this->_template,result, CV_TM_CCORR);
        double minValue, maxValue;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(result, &minValue, &maxValue, &minLoc, &maxLoc);

        if(maxValue>THRESHOLD)
        {
            std::cout<<"find a circle at    x="<<maxLoc.x<<"  y="<<maxLoc.y<<"    Value is:"<<maxValue<<std::endl;
            cv::rectangle(_img, maxLoc, cvPoint(maxLoc.x + this->_template.cols, maxLoc.y+ this->_template.rows), cvScalar(0,0,255),5);
        }
        else
            std::cout<<"can not find a circle"<<std::endl;
        cv::imshow(MATCH_WINDOW, _img);
        cv::imshow(TEMPLATE_WINDOW,this->_template);
        if('q'==cv::waitKey(3))
            exit(0);
    }
    void inital()
    {
        cv::Mat templateImg(120,120,CV_8UC3);
        cv::circle(templateImg,cv::Point(60,60),50,cv::Scalar(255,255,255));
        cv::cvtColor(templateImg,this->_template,CV_BGR2GRAY);
    }

private:
    cv::Mat _template;
};
ImageProcessor processor;

class ImageConverter
{
public:
  ImageConverter()
    : _it(_nh)
  {
    _image_sub = _it.subscribe(SUBSCRIBLE_TOPIC, 1,
      &ImageConverter::imageCb, this);
  }
  ~ImageConverter(){}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    processor.process(cv_ptr->image);
  }

private:
  ros::NodeHandle _nh;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _image_sub;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ImageProcessor");
  processor.inital();
  ImageConverter ic;
  ros::spin();
  return 0;
}

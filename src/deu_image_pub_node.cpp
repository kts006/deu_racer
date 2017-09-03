#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <cstdlib>
#include <unistd.h>

#define DISPLAY_VIDEO 0
#define LOOP_RATE 30

using namespace cv;

class CompressedImagePublisher {
public:
  CompressedImagePublisher();
  ~CompressedImagePublisher();
  CompressedImagePublisher(int device_id);
  CompressedImagePublisher(int device_id, int width, int height, int frame_rate, int brightness);
  void process();
  int getFrameRate() { return frame_rate; }

private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Publisher pub_raw_image;
  image_transport::Publisher pub_compressed_image;

  VideoCapture cap;
  Mat frame;
  sensor_msgs::ImagePtr msg;
  int frame_rate;

#if DISPLAY_VIDEO
  Mat edges;
#endif

  const char * topic_name = "/deu_racer/image_raw";
};

CompressedImagePublisher::CompressedImagePublisher() : nh("~"), it(nh) {
  std::string str_device_id, str_width, str_height, str_brightness;

  ROS_DEBUG("namespace = %s", nh.getNamespace().c_str());

  int device_id = -1, width = -1, height = -1;
  float brightness = 0.0;
  int frame_rate = 15;

  if (nh.getParam("device_id", device_id)) {
    ROS_INFO("device_id = %d", device_id);
  } else {
    ROS_INFO("The parameter \"device_id\" not found!");
  }
  nh.getParam("image_width", width);
  nh.getParam("image_height", height);
  nh.getParam("bright", brightness);
  nh.getParam("frame_rate", frame_rate);
  this->frame_rate = frame_rate;
  ROS_DEBUG("frame_rate = %d, width = %d, height = %d, brightness = %f", frame_rate, width, height, brightness);

  pub_raw_image = it.advertise(topic_name, 10);

  cap.open(device_id);
  if (!cap.isOpened()) {
    ROS_DEBUG("VideoCapture device not opend!");
    ros::shutdown();
  }
  cap.set(CAP_PROP_FRAME_WIDTH, width);
  cap.set(CAP_PROP_FRAME_HEIGHT, height);
  cap.set(CAP_PROP_FPS, frame_rate);
  cap.set(CAP_PROP_BRIGHTNESS, brightness);
}

CompressedImagePublisher::~CompressedImagePublisher() {
  std::cerr << "CompressedImagePublisher deleted..." << std::endl;
  cap.release();
}

CompressedImagePublisher::CompressedImagePublisher(int device_id) : it(nh) {
  pub_raw_image = it.advertise(topic_name, 1);

  cap.open(device_id);
  if (!cap.isOpened())
    ROS_DEBUG("VideoCapture device not opend!");
}

CompressedImagePublisher::CompressedImagePublisher(int device_id, int width, int height, int frame_rate, int brightness) : it(nh) {
  // CompressedImagePublisher(device_id);
  pub_raw_image = it.advertise(topic_name, 1);

  cap.open(device_id);
  if (cap.isOpened()) {
    cap.set(CAP_PROP_FRAME_WIDTH, width);
    cap.set(CAP_PROP_FRAME_HEIGHT, height);
    cap.set(CAP_PROP_FPS, frame_rate);
    cap.set(CAP_PROP_BRIGHTNESS, brightness);
  }
}

void CompressedImagePublisher::process() {
  cap >> frame;

#if DISPLAY_VIDEO
  cvtColor(frame, edges, COLOR_BGR2GRAY);
  GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
  Canny(edges, edges, 0, 30, 3);
  imshow("edges", edges);
#endif
  // https://answers.ros.org/question/203317/ros-opencv-segmantion-fault/
  // LJM 170812: to remove the segmentation fault
  //             DO NOT REMOVE THE FOLLOWING LINE!
  imwrite("test1.png", frame);

  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  pub_raw_image.publish(msg);
}


int main(int argc, char *argv[]) {
  ROS_INFO("deu_image_pub_node started...\n");
  // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  int device_id = 1;
  int loop_rate = LOOP_RATE;

  std::string ns = "~";
  ros::init(argc, argv, "deu_racer_image_pub");
  CompressedImagePublisher *image_pub = new CompressedImagePublisher();;
  loop_rate = image_pub->getFrameRate();
  // CompressedImagePublisher image_pub(device_id);
  // CompressedImagePublisher image_pub(device_id, 320, 240, LOOP_RATE, 0.7);

#if DISPLAY_VIDEO
  namedWindow("edges", WINDOW_AUTOSIZE);
  startWindowThread();
#endif

  ros::Rate rate(loop_rate);
  while (ros::ok()) {
    image_pub->process();

    // ros::spinOnce();
    rate.sleep();
  }
  delete image_pub;
  ROS_INFO("deu_image_pub_node terminated...\n");
  return 0;
}

#include <ros/ros.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace trinocular_view {

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class TrinocularView {
 public:
  TrinocularView();
  ~TrinocularView();

  void ImageCb(const ImageConstPtr& left_msg, const ImageConstPtr& middle_msg,
               const ImageConstPtr& right_msg);

 private:
  ros::NodeHandle pnh_;
  image_transport::SubscriberFilter left_sub_, middle_sub_, right_sub_;
  using ExactPolicy = ExactTime<Image, Image, Image>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;
  boost::shared_ptr<ExactSync> exact_sync_;
  int queue_size_{1};

  ImageConstPtr last_left_msg_, last_middle_msg_, last_right_msg_;
  cv::Mat last_left_image_, last_middle_image_, last_right_image_;
  boost::mutex image_mutex_;
};

TrinocularView::TrinocularView() {
  ros::NodeHandle pnh("~");

  // Resolve topic names
  const auto trinocular_ns = pnh.resolveName("trinocular");
  const auto image_resolved = pnh.resolveName("image");
  const auto left_topic =
      ros::names::clean(trinocular_ns + "/left/" + image_resolved);
  const auto middle_topic =
      ros::names::clean(trinocular_ns + "/middle/" + image_resolved);
  const auto right_topic =
      ros::names::clean(trinocular_ns + "/right/" + image_resolved);

  ROS_INFO("tirnocular ns: %s", trinocular_ns.c_str());
  ROS_INFO("image resolved: %s", image_resolved.c_str());
  ROS_INFO("left topic: %s", left_topic.c_str());
  ROS_INFO("middle topic: %s", middle_topic.c_str());
  ROS_INFO("right topic: %s", right_topic.c_str());

  // Subscribe to three input topics
  image_transport::ImageTransport it(pnh);
  left_sub_.subscribe(it, left_topic, 1);
  middle_sub_.subscribe(it, middle_topic, 1);
  right_sub_.subscribe(it, right_topic, 1);

  // Complain every 30s if the topics appear unsynchronized
  // TODO

  // Synchronize input topics
  exact_sync_ = boost::make_shared<ExactSync>(
      ExactPolicy(queue_size_), left_sub_, middle_sub_, right_sub_);
  exact_sync_->registerCallback(
      boost::bind(&TrinocularView::ImageCb, this, _1, _2, _3));

  // Do GUI window setup
  cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("middle", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("right", cv::WINDOW_AUTOSIZE);
}

TrinocularView::~TrinocularView() { cv::destroyAllWindows(); }

void TrinocularView::ImageCb(const ImageConstPtr& left_msg,
                             const ImageConstPtr& middle_msg,
                             const ImageConstPtr& right_msg) {
  // Hang on to image data for mouse callback
  last_left_msg_ = left_msg;
  last_middle_msg_ = middle_msg;
  last_right_msg_ = right_msg;

  // Convert to cv::Mat
  std::string view_encoding("bgr8");
  try {
    last_left_image_ = cv_bridge::toCvShare(left_msg, view_encoding)->image;
    last_middle_image_ = cv_bridge::toCvShare(middle_msg, view_encoding)->image;
    last_right_image_ = cv_bridge::toCvShare(right_msg, view_encoding)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Unable to convert one of '%s', '%s' or '%s' to '%s'",
              left_msg->encoding.c_str(), middle_msg->encoding.c_str(),
              right_msg->encoding.c_str(), view_encoding.c_str());
  }

  // View images
  if (!last_left_image_.empty()) {
    cv::imshow("left", last_left_image_);
  }
  if (!last_middle_image_.empty()) {
    cv::imshow("middle", last_middle_image_);
  }
  if (!last_right_image_.empty()) {
    cv::imshow("right", last_right_image_);
  }
}

}  // namespace trinocular_view

using namespace trinocular_view;

int main(int argc, char** argv) {
  ros::init(argc, argv, "trinocular_view");

  try {
    TrinocularView view;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
  }
}

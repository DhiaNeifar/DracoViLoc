// preview_node: compressed, rate-limited camera previews for the GUI.
//
// Input is the camera node's JPEG stream (/image/compressed, ~40 KB per frame,
// same stamps as /image). Publishes sensor_msgs/CompressedImage at most
// `max_fps`:
//
//   /dracoviloc/preview/raw/compressed      the camera JPEG, forwarded as is
//   /dracoviloc/preview/overlay/compressed  the same frame with YOLO boxes
//                                           (at most `max_width` pixels wide)
//
// The overlay is drawn here from /detections_output onto the exact camera
// frame the detections were computed from (matched by header stamp), so boxes
// never lag the image and the overlay runs at the detection rate. Detection
// coordinates are in the detector's encoded image.  Both raw and encoded
// camera intrinsics are used to undo either YOLO's centred letterbox or
// D-FINE's stretch before drawing on the raw frame.
//
// Raw 640x480 frames are ~1 MB each and the DDS transport here is UDP only,
// so nothing in the monitoring path touches /image. The GUI receives JPEGs
// that Qt decodes natively. Nothing is drawn or encoded while nobody watches.
#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "dracoviloc_monitor/detection_geometry.hpp"

using sensor_msgs::msg::CompressedImage;

namespace
{
int64_t nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

class PreviewNode : public rclcpp::Node
{
public:
  PreviewNode()
  : Node("dracoviloc_preview")
  {
    const auto raw_topic = declare_parameter<std::string>("raw_topic", "/image/compressed");
    const auto detections_topic =
      declare_parameter<std::string>("detections_topic", "/detections_output");
    const auto detection_info_topic = declare_parameter<std::string>(
      "detection_camera_info_topic", "/yolov8_encoder/resize/camera_info");
    const auto raw_info_topic =
      declare_parameter<std::string>("raw_camera_info_topic", "/camera_info");
    class_names_ = declare_parameter<std::vector<std::string>>("class_names", {"drone"});
    max_fps_ = declare_parameter<double>("max_fps", 15.0);
    max_width_ = declare_parameter<int>("max_width", 960);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
    if (max_fps_ <= 0.0 || max_width_ < 64 || jpeg_quality_ < 10 || jpeg_quality_ > 100) {
      throw std::invalid_argument("invalid preview parameters");
    }

    // Reliable with a 2-frame history: a busy subscriber (the Python GUI)
    // gets retransmissions instead of silent drops; the publisher never blocks.
    const auto preview_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    raw_pub_ = create_publisher<CompressedImage>("/dracoviloc/preview/raw/compressed", preview_qos);
    overlay_pub_ = create_publisher<CompressedImage>(
      "/dracoviloc/preview/overlay/compressed", preview_qos);
    raw_sub_ = create_subscription<CompressedImage>(
      raw_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](CompressedImage::ConstSharedPtr image) {on_image(image);});
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic, 10,
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr detections) {
        on_detections(*detections);
      });
    detection_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      detection_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        detection_camera_ = geometry(*info);
      });
    raw_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      raw_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        raw_camera_ = geometry(*info);
      });
    RCLCPP_INFO(get_logger(), "previews of %s (raw) and %s on %s (overlay), <= %.1f fps",
      raw_topic.c_str(), detections_topic.c_str(), raw_topic.c_str(), max_fps_);
  }

private:
  struct Throttle
  {
    rclcpp::Time last{0, 0, RCL_STEADY_TIME};
    // 20 % tolerance: a 15 fps camera must not be halved by frame jitter.
    bool due(const rclcpp::Time & now, double max_fps) const
    {
      return (now - last).seconds() >= 0.8 / max_fps;
    }
  };

  void on_image(const CompressedImage::ConstSharedPtr & image)
  {
    // Keep the last frames so detections can be drawn on their own frame.
    recent_.push_back(image);
    while (recent_.size() > 30) {  // ~2 s at 15 fps (~1 MB); detections arrive ~0.1 s later
      recent_.pop_front();
    }
    const auto now = steady_.now();
    if (raw_pub_->get_subscription_count() == 0 || !raw_throttle_.due(now, max_fps_)) {
      return;
    }
    raw_pub_->publish(*image);
    raw_throttle_.last = now;
  }

  void on_detections(const vision_msgs::msg::Detection2DArray & detections)
  {
    const auto now = steady_.now();
    if (overlay_pub_->get_subscription_count() == 0 || !overlay_throttle_.due(now, max_fps_)) {
      return;
    }
    const int64_t target = nanoseconds(detections.header.stamp);
    CompressedImage::ConstSharedPtr frame;
    for (auto it = recent_.rbegin(); it != recent_.rend(); ++it) {
      const int64_t stamp = nanoseconds((*it)->header.stamp);
      if (stamp == target) {
        frame = *it;
        break;
      }
      // Pipelines that restamp: fall back to the newest frame not after them.
      if (!frame && stamp <= target && target - stamp < 100000000LL) {
        frame = *it;
      }
    }
    if (!frame) {
      return;
    }
    try {
      cv::Mat canvas = cv::imdecode(frame->data, cv::IMREAD_COLOR);
      if (canvas.empty()) {
        throw std::runtime_error("cannot decode camera JPEG");
      }
      draw(canvas, detections);
      publish(*overlay_pub_, frame->header, canvas);
      overlay_throttle_.last = now;
    } catch (const std::exception & error) {
      warn(error);
    }
  }

  void draw(cv::Mat & canvas, const vision_msgs::msg::Detection2DArray & detections) const
  {
    if (!raw_camera_.valid() || !detection_camera_.valid()) {
      throw std::runtime_error("waiting for raw and detection camera_info");
    }
    const int thickness = std::max(2, (canvas.cols + canvas.rows) / 600);
    for (const auto & detection : detections.detections) {
      const auto & box = detection.bbox;
      const auto corners = dracoviloc_monitor::detection_box_to_raw(
        box.center.position.x, box.center.position.y, box.size_x, box.size_y,
        raw_camera_, detection_camera_);
      const cv::Point top_left(
        static_cast<int>(std::lround(corners.top_left.x)),
        static_cast<int>(std::lround(corners.top_left.y)));
      const cv::Point bottom_right(
        static_cast<int>(std::lround(corners.bottom_right.x)),
        static_cast<int>(std::lround(corners.bottom_right.y)));
      cv::rectangle(canvas, top_left, bottom_right, cv::Scalar(0, 255, 0), thickness);
      if (detection.results.empty()) {
        continue;
      }
      const auto & hypothesis = detection.results[0].hypothesis;
      std::size_t class_id = 0;
      try {
        class_id = static_cast<std::size_t>(std::stoul(hypothesis.class_id));
      } catch (const std::exception &) {
      }
      char label[96];
      std::snprintf(label, sizeof(label), "%s %.2f",
        class_id < class_names_.size() ? class_names_[class_id].c_str() : hypothesis.class_id.c_str(),
        hypothesis.score);
      const cv::Point origin(top_left.x, std::max(14, top_left.y - 4));
      cv::putText(canvas, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
    }
  }

  static dracoviloc_monitor::CameraGeometry geometry(
    const sensor_msgs::msg::CameraInfo & info)
  {
    return {info.width, info.height, info.k[0], info.k[4], info.k[2], info.k[5]};
  }

  void publish(rclcpp::Publisher<CompressedImage> & publisher,
    const std_msgs::msg::Header & header, cv::Mat frame)
  {
    if (frame.cols > max_width_) {
      const double scale = static_cast<double>(max_width_) / frame.cols;
      cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_AREA);
    }
    auto out = std::make_unique<CompressedImage>();
    out->header = header;
    out->format = "jpeg";
    cv::imencode(".jpg", frame, out->data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    publisher.publish(std::move(out));
  }

  void warn(const std::exception & error)
  {
    if (failures_++ % 100 == 0) {
      RCLCPP_WARN(get_logger(), "preview encoding failed: %s", error.what());
    }
  }

  std::vector<std::string> class_names_;
  double max_fps_;
  int max_width_;
  int jpeg_quality_;
  dracoviloc_monitor::CameraGeometry raw_camera_, detection_camera_;
  uint64_t failures_{0};
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  Throttle raw_throttle_, overlay_throttle_;
  std::deque<CompressedImage::ConstSharedPtr> recent_;
  rclcpp::Publisher<CompressedImage>::SharedPtr raw_pub_, overlay_pub_;
  rclcpp::Subscription<CompressedImage>::SharedPtr raw_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    detection_info_sub_, raw_info_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreviewNode>());
  rclcpp::shutdown();
  return 0;
}

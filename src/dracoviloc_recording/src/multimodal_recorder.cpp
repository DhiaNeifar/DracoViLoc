#include <audio_utils_msgs/msg/audio_frame.hpp>
#include <odas_ros_msgs/msg/odas_sst_array_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class MultimodalRecorder final : public rclcpp::Node
{
public:
  MultimodalRecorder()
  : Node("multimodal_recorder")
  {
    std::signal(SIGPIPE, SIG_IGN);
    const auto root = declare_parameter<std::string>("output_root", "runs");
    video_topic_ = declare_parameter<std::string>(
      "video_topic", "/yolov8_processed_image");
    audio_topic_ = declare_parameter<std::string>("audio_topic", "/sss");
    sst_topic_ = declare_parameter<std::string>("sst_topic", "/sst");
    video_fps_ = declare_parameter<double>("video_fps", 15.0);
    video_crf_ = declare_parameter<int>("video_crf", 23);
    require_video_ = declare_parameter<bool>("require_video", true);
    require_audio_ = declare_parameter<bool>("require_audio", true);
    require_sst_ = declare_parameter<bool>("require_sst", true);

    if (video_fps_ <= 0.0) {
      throw std::runtime_error("video_fps must be positive");
    }

    directory_ = fs::absolute(fs::path(root)) / timestamp();
    fs::create_directories(directory_);
    video_path_ = directory_ / "camera.mp4";
    audio_path_ = directory_ / "audio_sss.wav";
    sst_path_ = directory_ / "audio_sst.jsonl";

    video_sub_ = create_subscription<sensor_msgs::msg::Image>(
      video_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {record_video(*message);});

    const auto audio_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    audio_sub_ = create_subscription<audio_utils_msgs::msg::AudioFrame>(
      audio_topic_, audio_qos,
      [this](audio_utils_msgs::msg::AudioFrame::ConstSharedPtr message) {record_audio(*message);});

    sst_sub_ = create_subscription<odas_ros_msgs::msg::OdasSstArrayStamped>(
      sst_topic_, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](odas_ros_msgs::msg::OdasSstArrayStamped::ConstSharedPtr message) {
        record_sst(*message);
      });

    write_metadata("waiting_for_topics");
    RCLCPP_INFO(get_logger(), "Recording session created at %s", directory_.c_str());
    RCLCPP_INFO(get_logger(), "Waiting for%s%s%s before synchronized recording starts",
      require_video_ ? " video" : "", require_audio_ ? " /sss" : "",
      require_sst_ ? " /sst" : "");
  }

  ~MultimodalRecorder() override
  {
    close_video();
    close_audio();
    close_sst();
    write_metadata("complete");
    RCLCPP_INFO(get_logger(), "Recording saved in %s", directory_.c_str());
  }

private:
  static std::string timestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const auto raw = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&raw, &local);
    std::ostringstream output;
    output << std::put_time(&local, "%d_%m_%Y_%H_%M_%S");
    return output.str();
  }

  static std::string iso_timestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const auto raw = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&raw, &local);
    std::ostringstream output;
    output << std::put_time(&local, "%Y-%m-%dT%H:%M:%S%z");
    return output.str();
  }

  bool mark_stream_seen(bool & stream)
  {
    std::lock_guard<std::mutex> lock(start_mutex_);
    stream = true;
    if (!started_ && (!require_video_ || video_seen_) &&
      (!require_audio_ || audio_seen_) && (!require_sst_ || sst_seen_))
    {
      started_ = true;
      started_at_ = iso_timestamp();
      write_metadata("recording");
      RCLCPP_INFO(get_logger(), "All required topics are ready; synchronized recording started");
    }
    return started_;
  }

  template<typename T>
  static void write_little_endian(std::ostream & output, T value)
  {
    output.write(reinterpret_cast<const char *>(&value), sizeof(value));
  }

  void start_video(const sensor_msgs::msg::Image & message)
  {
    if (message.width == 0 || message.height == 0) {
      throw std::runtime_error("received a zero-sized video frame");
    }

    video_width_ = message.width;
    video_height_ = message.height;
    int descriptors[2];
    if (pipe(descriptors) != 0) {
      throw std::runtime_error(std::string("cannot create FFmpeg pipe: ") + std::strerror(errno));
    }

    video_pid_ = fork();
    if (video_pid_ < 0) {
      close(descriptors[0]);
      close(descriptors[1]);
      throw std::runtime_error(std::string("cannot start FFmpeg: ") + std::strerror(errno));
    }

    if (video_pid_ == 0) {
      setsid();
      std::signal(SIGINT, SIG_IGN);
      std::signal(SIGTERM, SIG_IGN);
      dup2(descriptors[0], STDIN_FILENO);
      close(descriptors[0]);
      close(descriptors[1]);

      const auto size = std::to_string(video_width_) + "x" + std::to_string(video_height_);
      const auto fps = std::to_string(video_fps_);
      const auto crf = std::to_string(video_crf_);
      execlp(
        "ffmpeg", "ffmpeg", "-hide_banner", "-loglevel", "warning", "-y",
        "-f", "rawvideo", "-pixel_format", "bgr24", "-video_size", size.c_str(),
        "-framerate", fps.c_str(), "-i", "pipe:0", "-an", "-c:v", "libx264",
        "-preset", "veryfast", "-crf", crf.c_str(), "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", video_path_.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }

    close(descriptors[0]);
    video_fd_ = descriptors[1];
    RCLCPP_INFO(get_logger(), "MP4 recording started: %ux%u at %.2f FPS",
      video_width_, video_height_, video_fps_);
  }

  static bool write_all(int descriptor, const uint8_t * data, size_t size)
  {
    while (size > 0) {
      const auto count = write(descriptor, data, size);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        return false;
      }
      data += count;
      size -= static_cast<size_t>(count);
    }
    return true;
  }

  void record_video(const sensor_msgs::msg::Image & message)
  {
    if (!mark_stream_seen(video_seen_)) {
      return;
    }
    std::lock_guard<std::mutex> lock(video_mutex_);
    if (video_failed_) {
      return;
    }

    try {
      if (video_fd_ < 0) {
        start_video(message);
      }
      if (message.width != video_width_ || message.height != video_height_) {
        throw std::runtime_error("video resolution changed during recording");
      }

      const size_t pixels = static_cast<size_t>(message.width) * message.height;
      video_buffer_.resize(pixels * 3);
      if (message.encoding == "bgr8") {
        for (uint32_t row = 0; row < message.height; ++row) {
          const auto * source = message.data.data() + static_cast<size_t>(row) * message.step;
          std::memcpy(video_buffer_.data() + static_cast<size_t>(row) * message.width * 3,
            source, static_cast<size_t>(message.width) * 3);
        }
      } else if (message.encoding == "rgb8") {
        for (uint32_t row = 0; row < message.height; ++row) {
          const auto * source = message.data.data() + static_cast<size_t>(row) * message.step;
          auto * destination = video_buffer_.data() + static_cast<size_t>(row) * message.width * 3;
          for (uint32_t column = 0; column < message.width; ++column) {
            destination[column * 3] = source[column * 3 + 2];
            destination[column * 3 + 1] = source[column * 3 + 1];
            destination[column * 3 + 2] = source[column * 3];
          }
        }
      } else {
        throw std::runtime_error("unsupported image encoding: " + message.encoding);
      }

      if (!write_all(video_fd_, video_buffer_.data(), video_buffer_.size())) {
        throw std::runtime_error("FFmpeg stopped accepting video frames");
      }
      ++video_frames_;
    } catch (const std::exception & error) {
      video_failed_ = true;
      RCLCPP_ERROR(get_logger(), "Video recording stopped: %s", error.what());
    }
  }

  void open_audio(const audio_utils_msgs::msg::AudioFrame & message)
  {
    if (message.format != "signed_16") {
      throw std::runtime_error("unsupported /sss format: " + message.format);
    }
    if (message.channel_count == 0 || message.sampling_frequency == 0) {
      throw std::runtime_error("invalid /sss audio metadata");
    }

    audio_channels_ = message.channel_count;
    audio_rate_ = message.sampling_frequency;
    audio_.open(audio_path_, std::ios::binary | std::ios::trunc);
    if (!audio_) {
      throw std::runtime_error("cannot create " + audio_path_.string());
    }
    std::array<char, 44> placeholder{};
    audio_.write(placeholder.data(), placeholder.size());
    RCLCPP_INFO(get_logger(), "WAV recording started: %u channels at %u Hz",
      audio_channels_, audio_rate_);
  }

  void record_audio(const audio_utils_msgs::msg::AudioFrame & message)
  {
    if (!mark_stream_seen(audio_seen_)) {
      return;
    }
    std::lock_guard<std::mutex> lock(audio_mutex_);
    if (audio_failed_) {
      return;
    }
    try {
      if (!audio_.is_open()) {
        open_audio(message);
      }
      if (message.format != "signed_16" || message.channel_count != audio_channels_ ||
        message.sampling_frequency != audio_rate_)
      {
        throw std::runtime_error("/sss audio format changed during recording");
      }
      audio_.write(reinterpret_cast<const char *>(message.data.data()), message.data.size());
      if (!audio_) {
        throw std::runtime_error("failed writing separated_audio.wav");
      }
      audio_bytes_ += message.data.size();
      ++audio_frames_;
    } catch (const std::exception & error) {
      audio_failed_ = true;
      RCLCPP_ERROR(get_logger(), "Audio recording stopped: %s", error.what());
    }
  }

  static std::string json_escape(const std::string & input)
  {
    std::ostringstream output;
    for (const char character : input) {
      switch (character) {
        case '\\': output << "\\\\"; break;
        case '"': output << "\\\""; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default: output << character; break;
      }
    }
    return output.str();
  }

  void record_sst(const odas_ros_msgs::msg::OdasSstArrayStamped & message)
  {
    if (!mark_stream_seen(sst_seen_)) {
      return;
    }
    std::lock_guard<std::mutex> lock(sst_mutex_);
    if (!sst_.is_open()) {
      sst_.open(sst_path_, std::ios::trunc);
      if (!sst_) {
        RCLCPP_ERROR(get_logger(), "Cannot create %s", sst_path_.c_str());
        sst_failed_ = true;
        return;
      }
    }
    sst_ << "{\"stamp\":{\"sec\":" << message.header.stamp.sec
         << ",\"nanosec\":" << message.header.stamp.nanosec << "},"
         << "\"frame_id\":\"" << json_escape(message.header.frame_id) << "\","
         << "\"sources\":[";
    for (size_t index = 0; index < message.sources.size(); ++index) {
      const auto & source = message.sources[index];
      if (index != 0) {
        sst_ << ',';
      }
      sst_ << "{\"slot\":" << index << ",\"id\":" << source.id
           << ",\"x\":" << source.x << ",\"y\":" << source.y
           << ",\"z\":" << source.z << ",\"activity\":" << source.activity << '}';
    }
    sst_ << "]}\n";
    if (!sst_) {
      RCLCPP_ERROR(get_logger(), "Failed writing audio_sst.jsonl");
      sst_failed_ = true;
      return;
    }
    ++sst_messages_;
  }

  void close_video()
  {
    std::lock_guard<std::mutex> lock(video_mutex_);
    if (video_fd_ >= 0) {
      close(video_fd_);
      video_fd_ = -1;
    }
    if (video_pid_ > 0) {
      int status = 0;
      waitpid(video_pid_, &status, 0);
      if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        RCLCPP_ERROR(get_logger(), "FFmpeg exited unsuccessfully (status=%d)", status);
        video_failed_ = true;
      }
      video_pid_ = -1;
    }
  }

  void close_audio()
  {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    if (!audio_.is_open()) {
      return;
    }

    const uint16_t format = 1;
    const uint16_t channels = static_cast<uint16_t>(audio_channels_);
    const uint32_t rate = audio_rate_;
    const uint16_t bits = 16;
    const uint16_t block_align = channels * bits / 8;
    const uint32_t byte_rate = rate * block_align;
    const uint32_t data_size = static_cast<uint32_t>(
      std::min<uint64_t>(audio_bytes_, UINT32_MAX - 36U));
    const uint32_t riff_size = data_size + 36;

    audio_.seekp(0);
    audio_.write("RIFF", 4);
    write_little_endian(audio_, riff_size);
    audio_.write("WAVEfmt ", 8);
    write_little_endian(audio_, uint32_t{16});
    write_little_endian(audio_, format);
    write_little_endian(audio_, channels);
    write_little_endian(audio_, rate);
    write_little_endian(audio_, byte_rate);
    write_little_endian(audio_, block_align);
    write_little_endian(audio_, bits);
    audio_.write("data", 4);
    write_little_endian(audio_, data_size);
    audio_.close();
  }

  void close_sst()
  {
    std::lock_guard<std::mutex> lock(sst_mutex_);
    if (sst_.is_open()) {
      sst_.close();
    }
  }

  void write_metadata(const std::string & state) const
  {
    std::ofstream output(directory_ / "metadata.json", std::ios::trunc);
    output << "{\n"
           << "  \"state\": \"" << state << "\",\n"
           << "  \"updated_at\": \"" << iso_timestamp() << "\",\n"
           << "  \"started_at\": \"" << started_at_ << "\",\n"
           << "  \"video_topic\": \"" << video_topic_ << "\",\n"
           << "  \"audio_topic\": \"" << audio_topic_ << "\",\n"
           << "  \"sst_topic\": \"" << sst_topic_ << "\",\n"
           << "  \"video_file\": \"camera.mp4\",\n"
           << "  \"audio_file\": \"audio_sss.wav\",\n"
           << "  \"sst_file\": \"audio_sst.jsonl\",\n"
           << "  \"video_fps\": " << video_fps_ << ",\n"
           << "  \"video_frames\": " << video_frames_.load() << ",\n"
           << "  \"audio_messages\": " << audio_frames_.load() << ",\n"
           << "  \"sst_messages\": " << sst_messages_.load() << ",\n"
           << "  \"video_ok\": " << (video_failed_ ? "false" : "true") << ",\n"
           << "  \"audio_ok\": " << (audio_failed_ ? "false" : "true") << ",\n"
           << "  \"sst_ok\": " << (sst_failed_ ? "false" : "true") << "\n"
           << "}\n";
  }

  fs::path directory_;
  fs::path video_path_;
  fs::path audio_path_;
  std::string video_topic_;
  std::string audio_topic_;
  std::string sst_topic_;
  double video_fps_{15.0};
  int video_crf_{23};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_sub_;
  rclcpp::Subscription<audio_utils_msgs::msg::AudioFrame>::SharedPtr audio_sub_;
  rclcpp::Subscription<odas_ros_msgs::msg::OdasSstArrayStamped>::SharedPtr sst_sub_;

  std::mutex start_mutex_;
  bool require_video_{true};
  bool require_audio_{true};
  bool require_sst_{true};
  bool video_seen_{false};
  bool audio_seen_{false};
  bool sst_seen_{false};
  bool started_{false};
  std::string started_at_;

  std::mutex video_mutex_;
  int video_fd_{-1};
  pid_t video_pid_{-1};
  uint32_t video_width_{0};
  uint32_t video_height_{0};
  std::vector<uint8_t> video_buffer_;
  std::atomic<uint64_t> video_frames_{0};
  bool video_failed_{false};

  std::mutex audio_mutex_;
  std::ofstream audio_;
  uint32_t audio_channels_{0};
  uint32_t audio_rate_{0};
  uint64_t audio_bytes_{0};
  std::atomic<uint64_t> audio_frames_{0};
  bool audio_failed_{false};

  std::mutex sst_mutex_;
  fs::path sst_path_;
  std::ofstream sst_;
  std::atomic<uint64_t> sst_messages_{0};
  bool sst_failed_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MultimodalRecorder>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("multimodal_recorder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

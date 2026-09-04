#include <alsa/asoundlib.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <csignal>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

class MultimodalRecorder final : public rclcpp::Node {
public:
  MultimodalRecorder() : Node("multimodal_recorder") {
    const auto root = declare_parameter<std::string>(
      "output_root", "/home/dhianeifar/DracoViLoc/runs");
    device_ = declare_parameter<std::string>("audio_device", "auto");
    rate_ = declare_parameter<int>("audio_rate", 44100);
    channels_ = declare_parameter<int>("audio_channels", 16);
    period_ = declare_parameter<int>("audio_period", 512);
    fps_ = declare_parameter<double>("video_fps", 15.0);
    bitrate_ = declare_parameter<int>("video_bitrate", 4000000);
    const auto video_topic = declare_parameter<std::string>(
      "video_topic", "/yolov8_processed_image");

    directory_ = fs::path(root) / timestamp();
    fs::create_directories(directory_);
    wav_path_ = directory_ / "audio.wav";
    video_path_ = directory_ / "video.mp4";
    try {
      open_audio();
      audio_thread_ = std::thread(&MultimodalRecorder::capture_audio, this);
      RCLCPP_INFO(get_logger(), "UMA16 capture: %s, %d channels, %d Hz",
        device_.c_str(), channels_, rate_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Audio recording unavailable: %s", error.what());
      if (pcm_) { snd_pcm_close(pcm_); pcm_ = nullptr; }
    }
    video_sub_ = create_subscription<sensor_msgs::msg::Image>(
      video_topic, rclcpp::SensorDataQoS(),
      std::bind(&MultimodalRecorder::video_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Recording into %s", directory_.c_str());
  }

  ~MultimodalRecorder() override {
    stop_.store(true);
    if (pcm_) snd_pcm_drop(pcm_);
    if (audio_thread_.joinable()) audio_thread_.join();
    if (pcm_) snd_pcm_close(pcm_);
    finalize_wav();
    close_video();
    RCLCPP_INFO(get_logger(), "Recording saved in %s", directory_.c_str());
  }

private:
  static std::string timestamp() {
    const auto raw = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local{};
    localtime_r(&raw, &local);
    std::ostringstream value;
    value << std::put_time(&local, "%d_%m_%Y_%H_%M_%S");
    return value.str();
  }

  template<typename T> static void put(std::ofstream & out, T value) {
    out.write(reinterpret_cast<const char *>(&value), sizeof(value));
  }

  static std::string find_uma16() {
    int card = -1;
    std::string usb_fallback;
    if (snd_card_next(&card) < 0) {
      throw std::runtime_error("Cannot enumerate ALSA sound cards");
    }
    while (card >= 0) {
      snd_ctl_t * control = nullptr;
      const std::string control_name = "hw:" + std::to_string(card);
      if (snd_ctl_open(&control, control_name.c_str(), 0) >= 0) {
        snd_ctl_card_info_t * info;
        snd_ctl_card_info_alloca(&info);
        if (snd_ctl_card_info(control, info) >= 0) {
          const std::string id = snd_ctl_card_info_get_id(info);
          const std::string name = snd_ctl_card_info_get_name(info);
          const std::string long_name = snd_ctl_card_info_get_longname(info);
          std::string description = id + " " + name + " " + long_name;
          std::transform(description.begin(), description.end(), description.begin(),
            [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
          if (description.find("uma16") != std::string::npos ||
            description.find("minidsp") != std::string::npos)
          {
            snd_ctl_close(control);
            return "hw:" + std::to_string(card) + ",0";
          }
          if (usb_fallback.empty() && description.find("usb") != std::string::npos) {
            usb_fallback = "hw:" + std::to_string(card) + ",0";
          }
        }
        snd_ctl_close(control);
      }
      if (snd_card_next(&card) < 0) break;
    }
    if (!usb_fallback.empty()) return usb_fallback;
    throw std::runtime_error(
      "UMA16 sound card was not found. Ensure /dev/snd is available in the container");
  }

  void open_audio() {
    if (device_ == "auto") device_ = find_uma16();
    int rc = snd_pcm_open(&pcm_, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) throw std::runtime_error("Cannot open UMA16 " + device_ + ": " + snd_strerror(rc));
    snd_pcm_hw_params_t * params;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_, params);
    snd_pcm_hw_params_set_access(pcm_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_, params, SND_PCM_FORMAT_S32_LE);
    unsigned int actual_rate = rate_;
    snd_pcm_hw_params_set_rate_near(pcm_, params, &actual_rate, nullptr);
    snd_pcm_hw_params_set_channels(pcm_, params, channels_);
    snd_pcm_uframes_t actual_period = period_;
    snd_pcm_hw_params_set_period_size_near(pcm_, params, &actual_period, nullptr);
    rc = snd_pcm_hw_params(pcm_, params);
    if (rc < 0) throw std::runtime_error("Cannot configure UMA16: " + std::string(snd_strerror(rc)));
    rate_ = actual_rate;
    period_ = actual_period;

    wav_.open(wav_path_, std::ios::binary | std::ios::trunc);
    if (!wav_) throw std::runtime_error("Cannot open " + wav_path_.string());
    wav_.write("RIFF", 4); put<uint32_t>(wav_, 0); wav_.write("WAVEfmt ", 8);
    put<uint32_t>(wav_, 16); put<uint16_t>(wav_, 1); put<uint16_t>(wav_, channels_);
    put<uint32_t>(wav_, rate_); put<uint32_t>(wav_, rate_ * channels_ * 4U);
    put<uint16_t>(wav_, channels_ * 4U); put<uint16_t>(wav_, 32);
    wav_.write("data", 4); put<uint32_t>(wav_, 0);
  }

  void capture_audio() {
    std::vector<int32_t> samples(static_cast<size_t>(period_) * channels_);
    while (!stop_.load()) {
      const auto frames = snd_pcm_readi(pcm_, samples.data(), period_);
      if (frames == -EPIPE) { snd_pcm_prepare(pcm_); continue; }
      if (frames < 0) {
        if (!stop_.load()) RCLCPP_ERROR(get_logger(), "UMA16 capture failed: %s", snd_strerror(frames));
        continue;
      }
      const size_t bytes = static_cast<size_t>(frames) * channels_ * sizeof(int32_t);
      wav_.write(reinterpret_cast<const char *>(samples.data()), bytes);
      audio_bytes_ += bytes;
    }
  }

  static std::string shell_quote(const std::string & value) {
    std::string result = "'";
    for (const char character : value) {
      if (character == '\'') result += "'\\''";
      else result += character;
    }
    return result + "'";
  }

  void open_video(const cv::Size & size) {
    if (std::system("command -v ffmpeg >/dev/null 2>&1") != 0)
      throw std::runtime_error("ffmpeg is not installed in the container");
    std::signal(SIGPIPE, SIG_IGN);
    std::ostringstream command;
    command << "ffmpeg -hide_banner -loglevel error -y "
            << "-f rawvideo -pixel_format bgr24 -video_size "
            << size.width << "x" << size.height << " -framerate " << fps_
            << " -i pipe:0 -an -c:v libx264 "
            << "-b:v " << bitrate_ << " -pix_fmt yuv420p -movflags +faststart "
            << shell_quote(video_path_.string());
    ffmpeg_ = popen(command.str().c_str(), "w");
    if (!ffmpeg_) throw std::runtime_error("Cannot start ffmpeg H.264 encoder");
    video_size_ = size;
    RCLCPP_INFO(get_logger(), "FFmpeg H.264 video: %dx%d at %.1f FPS",
      size.width, size.height, fps_);
  }

  void close_video() {
    if (!ffmpeg_) return;
    const int status = pclose(ffmpeg_);
    ffmpeg_ = nullptr;
    if (status != 0) RCLCPP_ERROR(get_logger(), "ffmpeg exited with status %d", status);
  }

  void video_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      if (msg->encoding != "bgr8" && msg->encoding != "rgb8")
        throw std::runtime_error("Expected bgr8/rgb8, received " + msg->encoding);
      const cv::Mat source(msg->height, msg->width, CV_8UC3, msg->data.data(), msg->step);
      cv::Mat converted;
      const cv::Mat * frame = &source;
      if (msg->encoding == "rgb8") {
        cv::cvtColor(source, converted, cv::COLOR_RGB2BGR);
        frame = &converted;
      }
      if (!ffmpeg_) open_video(frame->size());
      if (frame->size() != video_size_)
        throw std::runtime_error("Video resolution changed while recording");
      const size_t row_bytes = static_cast<size_t>(frame->cols) * 3U;
      for (int row = 0; row < frame->rows; ++row) {
        if (std::fwrite(frame->ptr(row), 1, row_bytes, ffmpeg_) != row_bytes)
          throw std::runtime_error("ffmpeg stopped accepting video frames");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
    }
  }

  void finalize_wav() {
    if (!wav_.is_open()) return;
    const auto size = static_cast<uint32_t>(std::min<uint64_t>(audio_bytes_, UINT32_MAX));
    wav_.seekp(4); put<uint32_t>(wav_, 36U + size);
    wav_.seekp(40); put(wav_, size);
    wav_.close();
  }

  fs::path directory_, wav_path_, video_path_;
  std::ofstream wav_;
  FILE * ffmpeg_{nullptr};
  cv::Size video_size_;
  snd_pcm_t * pcm_{nullptr};
  std::thread audio_thread_;
  std::atomic_bool stop_{false};
  std::string device_;
  double fps_{15.0};
  int bitrate_{4000000}, rate_{44100}, channels_{16};
  snd_pcm_uframes_t period_{512};
  uint64_t audio_bytes_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<MultimodalRecorder>()); }
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("multimodal_recorder"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

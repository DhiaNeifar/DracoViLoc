#include <audio_utils_msgs/msg/audio_frame.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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

class SssChannelRecorder final : public rclcpp::Node
{
public:
  SssChannelRecorder()
  : Node("sss_channel_recorder")
  {
    const auto root = declare_parameter<std::string>("output_root", "recordings");
    audio_topic_ = declare_parameter<std::string>("audio_topic", "/sss");
    const double gain_db = declare_parameter<double>("gain_db", 0.0);
    gain_ = static_cast<float>(std::pow(10.0, gain_db / 20.0));
    gain_db_ = gain_db;

    directory_ = fs::absolute(fs::path(root)) / timestamp();
    fs::create_directories(directory_);

    audio_sub_ = create_subscription<audio_utils_msgs::msg::AudioFrame>(
      audio_topic_, rclcpp::SensorDataQoS(),
      [this](audio_utils_msgs::msg::AudioFrame::ConstSharedPtr message) {record_audio(*message);});
  }

  ~SssChannelRecorder() override
  {
    close_audio();
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
    output << std::put_time(&local, "%Y-%m-%dT%H:%M:%S");
    return output.str();
  }

  template <typename T>
  static void write_little_endian(std::ostream & output, T value)
  {
    for (size_t index = 0; index < sizeof(T); ++index) {
      output.put(static_cast<char>((value >> (index * 8)) & 0xFF));
    }
  }

  void open_audio(const audio_utils_msgs::msg::AudioFrame & message)
  {
    if (message.format != "signed_16") {
      throw std::runtime_error("unsupported /sss format: " + message.format);
    }
    if (message.channel_count == 0 || message.sampling_frequency == 0 ||
      message.frame_sample_count == 0)
    {
      throw std::runtime_error("invalid /sss audio metadata");
    }

    audio_channels_ = message.channel_count;
    audio_rate_ = message.sampling_frequency;
    frame_samples_ = message.frame_sample_count;
    channel_bytes_ = static_cast<size_t>(frame_samples_) * sizeof(int16_t);
    scratch_.resize(frame_samples_);

    channels_.resize(audio_channels_);
    for (uint32_t channel = 0; channel < audio_channels_; ++channel) {
      const fs::path path = directory_ / ("channel_" + std::to_string(channel) + ".wav");
      channels_[channel].open(path, std::ios::binary | std::ios::trunc);
      if (!channels_[channel]) {
        throw std::runtime_error("cannot create " + path.string());
      }
      std::array<char, 44> placeholder{};
      channels_[channel].write(placeholder.data(), placeholder.size());
    }
    started_at_ = iso_timestamp();
    RCLCPP_INFO(
      get_logger(), "Per-channel WAV recording started: %u channels at %u Hz -> %s%s",
      audio_channels_, audio_rate_, directory_.c_str(),
      gain_db_ != 0.0 ? (" (gain " + std::to_string(gain_db_) + " dB)").c_str() : "");
  }

  void record_audio(const audio_utils_msgs::msg::AudioFrame & message)
  {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    if (audio_failed_) {
      return;
    }
    try {
      if (channels_.empty()) {
        open_audio(message);
      }
      if (message.format != "signed_16" || message.channel_count != audio_channels_ ||
        message.sampling_frequency != audio_rate_ ||
        message.frame_sample_count != frame_samples_)
      {
        throw std::runtime_error("/sss audio format changed during recording");
      }

      const size_t sample_count = message.data.size() / sizeof(int16_t);
      if (sample_count != static_cast<size_t>(frame_samples_) * audio_channels_) {
        throw std::runtime_error("/sss audio payload size mismatch");
      }
      const auto * samples = reinterpret_cast<const int16_t *>(message.data.data());
      for (uint32_t channel = 0; channel < audio_channels_; ++channel) {
        if (gain_ != 1.0F) {
          for (size_t frame = 0; frame < frame_samples_; ++frame) {
            const float scaled = static_cast<float>(samples[frame * audio_channels_ + channel]) * gain_;
            scratch_[frame] = static_cast<int16_t>(std::clamp(
              scaled, -32768.0F, 32767.0F));
          }
        } else {
          for (size_t frame = 0; frame < frame_samples_; ++frame) {
            scratch_[frame] = samples[frame * audio_channels_ + channel];
          }
        }
        channels_[channel].write(
          reinterpret_cast<const char *>(scratch_.data()), channel_bytes_);
        if (!channels_[channel]) {
          throw std::runtime_error("failed writing channel_" + std::to_string(channel) + ".wav");
        }
      }
      audio_samples_ += frame_samples_;
      ++audio_frames_;
    } catch (const std::exception & error) {
      audio_failed_ = true;
      RCLCPP_ERROR(get_logger(), "Channel recording stopped: %s", error.what());
    }
  }

  void close_audio()
  {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    if (channels_.empty() || !channels_.front().is_open()) {
      return;
    }

    const uint16_t format = 1;
    const uint16_t bits = 16;
    const uint16_t block_align = bits / 8;
    const uint32_t byte_rate = audio_rate_ * block_align;
    for (uint32_t channel = 0; channel < audio_channels_; ++channel) {
      const uint64_t written = channels_[channel].tellp();
      const uint32_t data_size = static_cast<uint32_t>(
        std::min<uint64_t>(written > 44 ? written - 44 : 0, UINT32_MAX - 36U));
      const uint32_t riff_size = data_size + 36;

      channels_[channel].seekp(0);
      channels_[channel].write("RIFF", 4);
      write_little_endian(channels_[channel], riff_size);
      channels_[channel].write("WAVEfmt ", 8);
      write_little_endian(channels_[channel], uint32_t{16});
      write_little_endian(channels_[channel], format);
      write_little_endian(channels_[channel], uint16_t{1});
      write_little_endian(channels_[channel], audio_rate_);
      write_little_endian(channels_[channel], byte_rate);
      write_little_endian(channels_[channel], block_align);
      write_little_endian(channels_[channel], bits);
      channels_[channel].write("data", 4);
      write_little_endian(channels_[channel], data_size);
      channels_[channel].close();
    }

    const double seconds = audio_rate_ > 0 ?
      static_cast<double>(audio_samples_) / audio_rate_ : 0.0;
    RCLCPP_INFO(
      get_logger(), "Per-channel WAV recording closed: %llu frames, %.1f s per channel%s",
      static_cast<unsigned long long>(audio_frames_.load()), seconds,
      audio_failed_ ? " (with errors)" : "");
    write_metadata(audio_failed_ ? "error" : "complete");
  }

  void write_metadata(const std::string & state) const
  {
    std::ofstream output(directory_ / "metadata.json", std::ios::trunc);
    output << "{\n"
           << "  \"state\": \"" << state << "\",\n"
           << "  \"updated_at\": \"" << iso_timestamp() << "\",\n"
           << "  \"started_at\": \"" << started_at_ << "\",\n"
           << "  \"audio_topic\": \"" << audio_topic_ << "\",\n"
           << "  \"audio_format\": \"signed_16\",\n"
           << "  \"channels\": " << audio_channels_ << ",\n"
           << "  \"sampling_frequency\": " << audio_rate_ << ",\n"
           << "  \"gain_db\": " << gain_db_ << ",\n"
           << "  \"audio_messages\": " << audio_frames_.load() << ",\n"
           << "  \"samples_per_channel\": " << audio_samples_ << "\n"
           << "}\n";
  }

  fs::path directory_;
  std::string audio_topic_;
  std::string started_at_;
  float gain_{1.0F};
  double gain_db_{0.0};

  rclcpp::Subscription<audio_utils_msgs::msg::AudioFrame>::SharedPtr audio_sub_;

  std::mutex audio_mutex_;
  std::vector<std::ofstream> channels_;
  std::vector<int16_t> scratch_;
  uint32_t audio_channels_{0};
  uint32_t audio_rate_{0};
  uint32_t frame_samples_{0};
  size_t channel_bytes_{0};
  uint64_t audio_samples_{0};
  std::atomic<uint64_t> audio_frames_{0};
  bool audio_failed_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SssChannelRecorder>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("sss_channel_recorder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

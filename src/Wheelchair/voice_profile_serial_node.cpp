#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cctype>

// Linux serial (termios)
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

class VoiceProfileSerialNode : public rclcpp::Node
{
public:
  VoiceProfileSerialNode()
  : Node("voice_profile_serial_node")
  {
    // ---------------- Parameters ----------------
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyS4");
    baud_ = this->declare_parameter<int>("baud", 115200);
    protocol_ = this->declare_parameter<std::string>("protocol", "byte"); // "byte" or "line"
    publish_topic_ = this->declare_parameter<std::string>("publish_topic", "/drive_profile");
    publish_hz_ = this->declare_parameter<double>("publish_hz", 2.0);

    // byte 协议下：两组单字节码映射到 mode/speed_level
    // 默认: mode 0/1/2 => 0x11/0x12/0x13; speed 0/1/2 => 0x21/0x22/0x23
    std::vector<int64_t> mode_codes64 =
      this->declare_parameter<std::vector<int64_t>>("mode_codes", {0x11, 0x12, 0x13});
    std::vector<int64_t> speed_codes64 =
      this->declare_parameter<std::vector<int64_t>>("speed_codes", {0x21, 0x22, 0x23});

    mode_codes_.clear();
    speed_codes_.clear();
    for (auto v : mode_codes64)  mode_codes_.push_back(static_cast<int>(v));
    for (auto v : speed_codes64) speed_codes_.push_back(static_cast<int>(v));

    if (mode_codes_.size() != 3 || speed_codes_.size() != 3) {
      RCLCPP_WARN(get_logger(),
        "mode_codes/speed_codes 建议长度为 3（对应 0/1/2）。当前 mode_codes=%zu speed_codes=%zu",
        mode_codes_.size(), speed_codes_.size());
    }

    // ---------------- Publisher QoS (Transient Local) ----------------
    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(publish_topic_, qos);

    // ---------------- State ----------------
    mode_ = 0;
    speed_level_ = 1; // 默认中速
    changed_ = true;

    // ---------------- Open serial ----------------
    if (!openSerial()) {
      RCLCPP_ERROR(get_logger(), "串口打开失败：%s (baud=%d)", port_.c_str(), baud_);
      // 不直接退出：允许你热插拔后重启节点或改参数
    } else {
      RCLCPP_INFO(get_logger(), "串口已打开：%s (baud=%d, protocol=%s)",
        port_.c_str(), baud_, protocol_.c_str());
    }

    // ---------------- Timers ----------------
    // 读串口更频繁一点降低延迟
    read_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&VoiceProfileSerialNode::readTimerCb, this));

    // 发布定时器（2Hz 默认），也会在 changed_ 时立刻输出（在 publishTimerCb 内判断）
    auto pub_period_ms = static_cast<int>(1000.0 / std::max(0.5, publish_hz_));
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(pub_period_ms),
      std::bind(&VoiceProfileSerialNode::publishTimerCb, this));
  }

  ~VoiceProfileSerialNode() override
  {
    closeSerial();
  }

private:
  // -------- Serial helpers --------
  static speed_t baudToTermios(int baud)
  {
    switch (baud) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 500000: return B500000;
      case 921600: return B921600;
      default: return B115200;
    }
  }

  bool openSerial()
  {
    closeSerial();

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port_.c_str(), strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      closeSerial();
      return false;
    }

    // Raw mode
    cfmakeraw(&tty);

    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // Enable receiver, ignore modem control lines
    tty.c_cflag |= (CLOCAL | CREAD);

    // Non-block read
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 0.1s

    speed_t spd = baudToTermios(baud_);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      closeSerial();
      return false;
    }

    // flush
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void closeSerial()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // -------- Parsing & state update --------
  void handleByte(uint8_t v)
  {
    // mode
    auto it_m = std::find(mode_codes_.begin(), mode_codes_.end(), static_cast<int>(v));
    if (it_m != mode_codes_.end()) {
      int new_mode = static_cast<int>(std::distance(mode_codes_.begin(), it_m));
      setProfile(new_mode, speed_level_);
      return;
    }
    // speed
    auto it_s = std::find(speed_codes_.begin(), speed_codes_.end(), static_cast<int>(v));
    if (it_s != speed_codes_.end()) {
      int new_speed = static_cast<int>(std::distance(speed_codes_.begin(), it_s));
      setProfile(mode_, new_speed);
      return;
    }
  }

  static std::string stripSpaces(const std::string &in)
  {
    std::string out;
    out.reserve(in.size());
    for (char c : in) if (!std::isspace(static_cast<unsigned char>(c))) out.push_back(c);
    return out;
  }

  // line 协议：支持 "M1S2" / "M1 S2" / "1,2"
  void handleLine(const std::string &line_raw)
  {
    std::string s = stripSpaces(line_raw);
    if (s.empty()) return;

    // 去掉末尾 \r
    if (!s.empty() && s.back() == '\r') s.pop_back();
    if (s.empty()) return;

    // 1) MxSy
    auto posM = s.find('M');
    auto posS = s.find('S');
    if (posM != std::string::npos && posS != std::string::npos && posS > posM) {
      try {
        int m = std::stoi(s.substr(posM + 1, posS - (posM + 1)));
        int v = std::stoi(s.substr(posS + 1));
        m = std::max(0, std::min(2, m));
        v = std::max(0, std::min(2, v));
        setProfile(m, v);
        return;
      } catch (...) {
        return;
      }
    }

    // 2) "mode,speed"
    auto comma = s.find(',');
    if (comma != std::string::npos) {
      try {
        int m = std::stoi(s.substr(0, comma));
        int v = std::stoi(s.substr(comma + 1));
        m = std::max(0, std::min(2, m));
        v = std::max(0, std::min(2, v));
        setProfile(m, v);
      } catch (...) {
        return;
      }
    }
  }

  void setProfile(int new_mode, int new_speed)
  {
    new_mode = std::max(0, std::min(2, new_mode));
    new_speed = std::max(0, std::min(2, new_speed));

    std::lock_guard<std::mutex> lk(mtx_);
    if (new_mode != mode_ || new_speed != speed_level_) {
      mode_ = new_mode;
      speed_level_ = new_speed;
      changed_ = true;
    }
  }

  // -------- Timers --------
  void readTimerCb()
  {
    if (fd_ < 0) {
      // 尝试重连（可选）
      static int cnt = 0;
      if (++cnt % 50 == 0) { // 大约 1s 一次
        openSerial();
      }
      return;
    }

    int bytes_available = 0;
    if (ioctl(fd_, FIONREAD, &bytes_available) != 0) {
      return;
    }
    if (bytes_available <= 0) return;

    // 读出可用数据
    std::vector<uint8_t> buf(static_cast<size_t>(bytes_available));
    ssize_t n = ::read(fd_, buf.data(), buf.size());
    if (n <= 0) return;

    if (protocol_ == "line") {
      for (ssize_t i = 0; i < n; ++i) {
        char c = static_cast<char>(buf[static_cast<size_t>(i)]);
        if (c == '\n') {
          handleLine(line_buffer_);
          line_buffer_.clear();
        } else {
          line_buffer_.push_back(c);
          // 防止异常长行占用内存
          if (line_buffer_.size() > 256) {
            line_buffer_.clear();
          }
        }
      }
    } else {
      for (ssize_t i = 0; i < n; ++i) {
        handleByte(buf[static_cast<size_t>(i)]);
      }
    }
  }

  void publishTimerCb()
  {
    int m, s;
    bool changed;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      m = mode_;
      s = speed_level_;
      changed = changed_;
      changed_ = false;
    }

    std_msgs::msg::Int32MultiArray msg;
    msg.data = {m, s};
    pub_->publish(msg);

    if (changed) {
      RCLCPP_INFO(get_logger(), "Profile updated: mode=%d speed_level=%d", m, s);
    }
  }

private:
    // params
    std::string port_;
    int baud_;
    std::string protocol_;
    std::string publish_topic_;
    double publish_hz_;
    std::vector<int> mode_codes_;
    std::vector<int> speed_codes_;

    // ros
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // serial
    int fd_{-1};
    std::string line_buffer_;

    // state
    std::mutex mtx_;
    int mode_{0};
    int speed_level_{1};
    bool changed_{true};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoiceProfileSerialNode>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {
constexpr std::uint8_t COMM_FW_VERSION = 0;
constexpr std::uint8_t COMM_GET_VALUES = 4;
constexpr std::uint8_t COMM_SET_DUTY = 5;
constexpr std::uint8_t COMM_SET_CURRENT = 6;
constexpr std::uint8_t COMM_SET_CURRENT_BRAKE = 7;
constexpr std::uint8_t COMM_SET_RPM = 8;
constexpr std::uint8_t COMM_SET_POS = 9;
constexpr std::uint8_t COMM_SET_HANDBRAKE = 10;
constexpr std::uint8_t COMM_GET_MCCONF = 14;
constexpr std::uint8_t COMM_GET_MCCONF_DEFAULT = 15;
constexpr std::uint8_t COMM_GET_APPCONF = 17;
constexpr std::uint8_t COMM_GET_APPCONF_DEFAULT = 18;
constexpr std::uint8_t COMM_TERMINAL_CMD = 20;
constexpr std::uint8_t COMM_DETECT_ENCODER = 27;
constexpr std::uint8_t COMM_DETECT_HALL_FOC = 28;
constexpr std::uint8_t COMM_REBOOT = 29;
constexpr std::uint8_t COMM_ALIVE = 30;
constexpr std::uint8_t COMM_FORWARD_CAN = 34;
constexpr std::uint8_t RIGHT_ID = 2;

rclcpp::QoS stateQos() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); }

std::uint16_t crc16(const std::uint8_t *data, std::size_t len) {
  std::uint16_t crc = 0U;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

void appendI32(std::vector<std::uint8_t> &out, std::int32_t value) {
  const auto u = static_cast<std::uint32_t>(value);
  out.push_back(static_cast<std::uint8_t>(u >> 24U));
  out.push_back(static_cast<std::uint8_t>(u >> 16U));
  out.push_back(static_cast<std::uint8_t>(u >> 8U));
  out.push_back(static_cast<std::uint8_t>(u));
}

std::int16_t i16(const std::uint8_t *p) {
  return static_cast<std::int16_t>((static_cast<std::uint16_t>(p[0]) << 8U) | p[1]);
}
std::int32_t i32(const std::uint8_t *p) {
  return static_cast<std::int32_t>((static_cast<std::uint32_t>(p[0]) << 24U) |
    (static_cast<std::uint32_t>(p[1]) << 16U) | (static_cast<std::uint32_t>(p[2]) << 8U) | p[3]);
}

std::vector<std::uint8_t> frame(const std::vector<std::uint8_t> &payload) {
  std::vector<std::uint8_t> out;
  if (payload.empty() || payload.size() > 65535U) return out;
  if (payload.size() <= 255U) {
    out.reserve(payload.size() + 5U); out.push_back(2U); out.push_back(static_cast<std::uint8_t>(payload.size()));
  } else {
    out.reserve(payload.size() + 6U); out.push_back(3U);
    out.push_back(static_cast<std::uint8_t>(payload.size() >> 8U)); out.push_back(static_cast<std::uint8_t>(payload.size()));
  }
  out.insert(out.end(), payload.begin(), payload.end());
  const auto c = crc16(payload.data(), payload.size());
  out.push_back(static_cast<std::uint8_t>(c >> 8U)); out.push_back(static_cast<std::uint8_t>(c)); out.push_back(3U);
  return out;
}

bool containsValidVescFrame(const std::vector<std::uint8_t> &bytes) {
  for (std::size_t s = 0; s < bytes.size(); ++s) {
    const std::uint8_t start = bytes[s];
    std::size_t h = 0, n = 0;
    if (start == 2U) {
      if (bytes.size() - s < 2U) continue;
      h = 2U; n = bytes[s + 1U];
    } else if (start == 3U) {
      if (bytes.size() - s < 3U) continue;
      h = 3U; n = (std::size_t(bytes[s + 1U]) << 8U) | bytes[s + 2U];
    } else if (start == 4U) {
      if (bytes.size() - s < 4U) continue;
      h = 4U; n = (std::size_t(bytes[s + 1U]) << 16U) |
                  (std::size_t(bytes[s + 2U]) << 8U) | bytes[s + 3U];
    } else {
      continue;
    }
    if (n == 0U || n > 4096U) continue;
    const std::size_t total = h + n + 3U;
    if (bytes.size() - s < total || bytes[s + total - 1U] != 3U) continue;
    const auto expected = static_cast<std::uint16_t>((std::uint16_t(bytes[s + h + n]) << 8U) |
                                                     bytes[s + h + n + 1U]);
    if (crc16(bytes.data() + s + h, n) == expected) return true;
  }
  return false;
}

// Extract exactly one complete VESC frame from a TCP byte stream. Keeping packet
// boundaries intact is important for realtime traffic: the F411 text gateway can
// then forward small control/telemetry frames in one USB command instead of
// re-chunking an arbitrary aggregate and inserting pacing delays mid-packet.
bool popValidVescFrame(std::vector<std::uint8_t> &bytes, std::vector<std::uint8_t> *out) {
  if (!out) return false;
  out->clear();
  while (!bytes.empty()) {
    const std::uint8_t start = bytes[0];
    std::size_t h = 0U, n = 0U;
    if (start == 2U) {
      if (bytes.size() < 2U) return false;
      h = 2U; n = bytes[1];
    } else if (start == 3U) {
      if (bytes.size() < 3U) return false;
      h = 3U; n = (std::size_t(bytes[1]) << 8U) | bytes[2];
    } else if (start == 4U) {
      if (bytes.size() < 4U) return false;
      h = 4U; n = (std::size_t(bytes[1]) << 16U) |
                  (std::size_t(bytes[2]) << 8U) | bytes[3];
    } else {
      bytes.erase(bytes.begin());
      continue;
    }
    if (n == 0U || n > 4096U) { bytes.erase(bytes.begin()); continue; }
    const std::size_t total = h + n + 3U;
    if (bytes.size() < total) return false;
    const auto expected = static_cast<std::uint16_t>((std::uint16_t(bytes[h + n]) << 8U) |
                                                     bytes[h + n + 1U]);
    if (bytes[total - 1U] != 3U || crc16(bytes.data() + h, n) != expected) {
      bytes.erase(bytes.begin());
      continue;
    }
    out->assign(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(total));
    bytes.erase(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(total));
    return true;
  }
  return false;
}

std::vector<std::uint8_t> motorPayload(int motor, std::vector<std::uint8_t> inner) {
  if (motor == 1) return inner;
  std::vector<std::uint8_t> out{COMM_FORWARD_CAN, RIGHT_ID};
  out.insert(out.end(), inner.begin(), inner.end());
  return out;
}

std::string hex(const std::vector<std::uint8_t> &data) {
  std::ostringstream o; o << std::hex << std::uppercase << std::setfill('0');
  for (const auto b : data) o << std::setw(2) << static_cast<unsigned>(b);
  return o.str();
}

int nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool unhex(const std::string &s, std::vector<std::uint8_t> *out) {
  if (!out || s.empty() || (s.size() & 1U) || s.size() > 8192U) return false;
  out->clear(); out->reserve(s.size() / 2U);
  for (std::size_t i = 0; i < s.size(); i += 2U) {
    const int a = nibble(s[i]), b = nibble(s[i + 1U]);
    if (a < 0 || b < 0) { out->clear(); return false; }
    out->push_back(static_cast<std::uint8_t>((a << 4) | b));
  }
  return true;
}

std::vector<std::string> split(const std::string &s, char sep) {
  std::vector<std::string> out; std::stringstream ss(s); std::string item;
  while (std::getline(ss, item, sep)) out.push_back(item);
  return out;
}

bool parseMotor(const std::string &s, int *motor) {
  if (!motor || (s != "1" && s != "2")) return false;
  *motor = s == "2" ? 2 : 1; return true;
}

bool parseDouble(const std::string &s, double *value) {
  if (!value) return false;
  char *end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno || end == s.c_str() || *end != '\0' || !std::isfinite(v)) return false;
  *value = v; return true;
}
}  // namespace

class VescToolBridge final : public rclcpp::Node {
 public:
  VescToolBridge() : Node("vesc_tool_bridge") {
    poll_hz_ = std::clamp(declare_parameter<double>("maintenance_poll_hz", 10.0), 1.0, 25.0);
    tcp_service_hz_ = std::clamp(declare_parameter<double>("tcp_service_hz", 1000.0), 100.0, 2000.0);
    max_abs_duty_ = std::clamp(declare_parameter<double>("max_abs_duty", 0.95), 0.01, 0.99);
    max_abs_current_a_ = std::clamp(declare_parameter<double>("max_abs_current_a", 20.0), 0.1, 100.0);
    max_abs_rpm_ = std::clamp(declare_parameter<double>("max_abs_rpm", 10000.0), 10.0, 200000.0);
    tcp_enabled_ = declare_parameter<bool>("tcp_enabled", true);
    tcp_port_ = static_cast<int>(std::clamp<std::int64_t>(declare_parameter<int>("tcp_port", 65102), 1024, 65535));
    python_tcp_enabled_ = declare_parameter<bool>("python_tcp_enabled", true);
    python_tcp_port_ = static_cast<int>(std::clamp<std::int64_t>(declare_parameter<int>("python_tcp_port", 65101), 1024, 65535));
    if (python_tcp_port_ == tcp_port_) throw std::runtime_error("python_tcp_port and tcp_port must differ");

    tx_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/stmf4/vesc/maintenance_tx", rclcpp::QoS(100).reliable());
    runtime_probe_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/stmf4/vesc/runtime_tx", rclcpp::QoS(20).reliable());
    mode_pub_ = create_publisher<std_msgs::msg::String>("/stmf4/vesc/mode", stateQos());
    active_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/vesc/maintenance_active", stateQos());
    owner_pub_ = create_publisher<std_msgs::msg::String>("/esc/vesc/maintenance_owner", stateQos());
    status_pub_ = create_publisher<std_msgs::msg::String>("/esc/vesc/tool_status", stateQos());
    telemetry_pub_ = create_publisher<std_msgs::msg::String>("/esc/vesc/tool_telemetry", stateQos());
    raw_pub_ = create_publisher<std_msgs::msg::String>("/esc/vesc/raw_reply", rclcpp::QoS(20).reliable());

    rx_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>("/stmf4/vesc/rx", rclcpp::QoS(100).reliable(),
      [this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr m) { consume(m->data); });
    transport_sub_ = create_subscription<std_msgs::msg::Bool>("/stmf4/vesc/connected", stateQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { transport_connected_ = m->data; publishStatus(); });
    speed_sub_ = create_subscription<std_msgs::msg::Float64>("/esc/drive_actual_mps", 10,
      [this](std_msgs::msg::Float64::ConstSharedPtr m) { if (std::isfinite(m->data)) speed_mps_ = m->data; });
    mux_sub_ = create_subscription<std_msgs::msg::String>("/esc/mux/active_source", stateQos(),
      [this](std_msgs::msg::String::ConstSharedPtr m) { mux_source_ = m->data; });
    command_sub_ = create_subscription<std_msgs::msg::String>("/esc/vesc/tool_command", 20,
      [this](std_msgs::msg::String::ConstSharedPtr m) { command(m->data); });

    transition_timer_ = create_wall_timer(20ms, [this]() { transitionTick(); });
    poll_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / poll_hz_)), [this]() { pollTick(); });
    tcp_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / tcp_service_hz_)), [this]() { tcpTick(); pythonTcpTick(); });
    if (tcp_enabled_) setupTcpServer();
    if (python_tcp_enabled_) setupPythonTcpServer();
    publishMode("RUNTIME");
    publishActive(false); publishOwner("RUNTIME"); publishStatus("runtime");
    RCLCPP_INFO(get_logger(),
      "VESC maintenance arbiter ready | Python(priority=100)=%s 127.0.0.1:%d | VESC Tool(priority=90)=%s 127.0.0.1:%d",
      python_tcp_enabled_ ? "ON" : "OFF", python_tcp_port_, tcp_enabled_ ? "ON" : "OFF", tcp_port_);
  }

  ~VescToolBridge() override {
    closePythonTcpClient(); closeTcpClient();
    if (python_tcp_server_fd_ >= 0) ::close(python_tcp_server_fd_);
    if (tcp_server_fd_ >= 0) ::close(tcp_server_fd_);
  }

 private:
  enum class Transition { NONE, ENTER, ENTER_ROUTE, EXIT_STOP, EXIT_ROUTE };

  void setupTcpServer() {
    tcp_server_fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (tcp_server_fd_ < 0) { RCLCPP_ERROR(get_logger(), "VESC TCP socket failed: %s", std::strerror(errno)); return; }
    int one = 1;
    (void)::setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<std::uint16_t>(tcp_port_));
    if (::bind(tcp_server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 || ::listen(tcp_server_fd_, 16) != 0) {
      RCLCPP_ERROR(get_logger(), "VESC TCP bind/listen 127.0.0.1:%d failed: %s", tcp_port_, std::strerror(errno));
      ::close(tcp_server_fd_); tcp_server_fd_ = -1; return;
    }
  }

  void setupPythonTcpServer() {
    python_tcp_server_fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (python_tcp_server_fd_ < 0) { RCLCPP_ERROR(get_logger(), "Python maintenance TCP socket failed: %s", std::strerror(errno)); return; }
    int one = 1;
    (void)::setsockopt(python_tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<std::uint16_t>(python_tcp_port_));
    if (::bind(python_tcp_server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
        ::listen(python_tcp_server_fd_, 16) != 0) {
      RCLCPP_ERROR(get_logger(), "Python maintenance TCP bind/listen 127.0.0.1:%d failed: %s", python_tcp_port_, std::strerror(errno));
      ::close(python_tcp_server_fd_); python_tcp_server_fd_ = -1;
    }
  }

  void closeTcpClient() {
    if (tcp_client_fd_ >= 0) { ::shutdown(tcp_client_fd_, SHUT_RDWR); ::close(tcp_client_fd_); }
    tcp_client_fd_ = -1;
    tcp_client_armed_ = false;
    tcp_probe_pending_ = false;
    tcp_pending_rx_.clear();
    tcp_pending_tx_.clear();
  }

  void closePythonTcpClient() {
    if (python_tcp_client_fd_ >= 0) { ::shutdown(python_tcp_client_fd_, SHUT_RDWR); ::close(python_tcp_client_fd_); }
    python_tcp_client_fd_ = -1;
    python_tcp_client_armed_ = false;
    python_probe_pending_ = false;
    python_pending_rx_.clear();
    python_pending_tx_.clear();
  }

  const char *maintenanceOwner() const {
    if (python_tcp_client_armed_) return "PYTHON_MAINTENANCE";
    if (tcp_client_armed_) return "VESC_TOOL";
    return maintenance_active_ ? "MAINTENANCE_API" : "RUNTIME";
  }

  void publishOwner(const char *owner = nullptr) {
    if (!owner_pub_) return;
    std_msgs::msg::String m; m.data = owner ? owner : maintenanceOwner(); owner_pub_->publish(m);
  }

  bool vehicleIdleForMaintenance() const {
    // Maintenance is intentionally above VESC Tool/manual/perception/Nav2.
    // A lower-priority source may remain logically selected while commanding
    // zero; ownership is safe to preempt when measured vehicle speed is stopped.
    return std::abs(speed_mps_) <= 0.03;
  }

  void sendRuntimeProbe() {
    if (!runtime_probe_pub_) return;
    std_msgs::msg::UInt8MultiArray m;
    m.data = frame({COMM_FW_VERSION});
    runtime_probe_pub_->publish(m);
  }

  void sendMaintenanceSafeStop() {
    for (int motor = 1; motor <= 2; ++motor) {
      std::vector<std::uint8_t> p{COMM_SET_CURRENT}; appendI32(p, 0);
      const auto packet = frame(motorPayload(motor, std::move(p)));
      if (!packet.empty()) { std_msgs::msg::UInt8MultiArray m; m.data = packet; tx_pub_->publish(m); }
    }
  }

  void forceRuntimeAfterTcp(const std::string &event) {
    const bool was_armed = tcp_client_armed_;
    closeTcpClient();
    if (!was_armed) {
      publishOwner();
      publishStatus(event + "_unarmed");
      return;
    }
    // Python maintenance is a higher-priority owner. Losing the lower-priority
    // VESC Tool socket must never tear Python out of MAINTENANCE.
    if (python_tcp_client_armed_) {
      publishOwner("PYTHON_MAINTENANCE");
      publishStatus(event + "_python_kept");
      return;
    }
    if (transition_ == Transition::ENTER || transition_ == Transition::ENTER_ROUTE) {
      publishMode("RUNTIME"); publishActive(false); publishOwner("RUNTIME"); transition_ = Transition::NONE;
    } else if (maintenance_active_) {
      sendMaintenanceSafeStop(); transition_ = Transition::EXIT_STOP;
      transition_at_ = std::chrono::steady_clock::now() + 100ms;
    }
    publishStatus(event);
  }

  void queueTcpTx(const std::vector<std::uint8_t> &bytes) {
    if (tcp_client_fd_ < 0 || bytes.empty()) return;
    if (tcp_pending_tx_.size() + bytes.size() > 65536U) {
      forceRuntimeAfterTcp("tcp_tx_overflow"); return;
    }
    tcp_pending_tx_.insert(tcp_pending_tx_.end(), bytes.begin(), bytes.end());
  }

  void flushTcpTx() {
    while (tcp_client_fd_ >= 0 && !tcp_pending_tx_.empty()) {
      const ssize_t n = ::send(tcp_client_fd_, tcp_pending_tx_.data(), tcp_pending_tx_.size(), MSG_NOSIGNAL);
      if (n > 0) { tcp_pending_tx_.erase(tcp_pending_tx_.begin(), tcp_pending_tx_.begin() + n); continue; }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
      forceRuntimeAfterTcp("tcp_send_disconnected"); return;
    }
  }

  void forwardTcpPendingRx() {
    if (python_tcp_client_armed_) { tcp_pending_rx_.clear(); return; }
    if (tcp_client_fd_ < 0 || !tcp_client_armed_ || !maintenance_active_ ||
        transition_ != Transition::NONE || tcp_pending_rx_.empty()) return;
    // Preserve VESC packet boundaries. Bound each 10-ms service slice so a
    // firmware/config burst cannot starve ROS callbacks, while realtime small
    // frames (50-Hz control/telemetry) normally drain in the same slice.
    for (std::size_t budget = 0U; budget < 32U; ++budget) {
      std::vector<std::uint8_t> packet;
      if (!popValidVescFrame(tcp_pending_rx_, &packet)) break;
      std_msgs::msg::UInt8MultiArray m; m.data = std::move(packet); tx_pub_->publish(m);
    }
  }

  void tcpTick() {
    if (tcp_server_fd_ < 0) return;
    if (tcp_client_fd_ < 0) {
      sockaddr_in peer{}; socklen_t peer_len = sizeof(peer);
      const int fd = ::accept4(tcp_server_fd_, reinterpret_cast<sockaddr *>(&peer), &peer_len,
                               SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd >= 0) {
        int one = 1; (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        tcp_client_fd_ = fd;
        tcp_client_armed_ = false;
        tcp_probe_pending_ = false;
        tcp_pending_rx_.clear(); tcp_pending_tx_.clear();
        tcp_client_handshake_deadline_ = std::chrono::steady_clock::now() + 500ms;
        publishOwner();
        publishStatus("vesc_tcp_connected_waiting_valid_frame");
      }
    }
    if (tcp_client_fd_ < 0) return;

    std::uint8_t buf[1024];
    for (;;) {
      const ssize_t n = ::recv(tcp_client_fd_, buf, sizeof(buf), 0);
      if (n > 0) {
        if (tcp_pending_rx_.size() + static_cast<std::size_t>(n) > 65536U) {
          forceRuntimeAfterTcp("tcp_rx_overflow"); return;
        }
        tcp_pending_rx_.insert(tcp_pending_rx_.end(), buf, buf + n);
        continue;
      }
      if (n == 0) { forceRuntimeAfterTcp("tcp_client_disconnected"); return; }
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      forceRuntimeAfterTcp("tcp_recv_error"); return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!tcp_client_armed_) {
      if (containsValidVescFrame(tcp_pending_rx_)) {
        if (python_tcp_client_armed_ && maintenance_active_) {
          // Valid VESC Tool client may remain connected underneath Python, but
          // its first packet and every packet during Python ownership are dropped.
          tcp_client_armed_ = true;
          tcp_pending_rx_.clear(); tcp_pending_tx_.clear();
          publishOwner();
          publishStatus("vesc_tcp_valid_suspended_by_python");
        } else if (!vehicleIdleForMaintenance() || transition_ != Transition::NONE || maintenance_active_) {
          forceRuntimeAfterTcp("tcp_valid_frame_rejected_vehicle_not_idle_or_busy");
          return;
        } else {
          tcp_client_armed_ = true;
          publishOwner();
          if (transport_connected_) {
            beginEnter();
            publishStatus("tcp_valid_frame_entering_maintenance");
          } else {
            tcp_probe_pending_ = true;
            tcp_probe_deadline_ = now + 1500ms;
            tcp_probe_next_ = now;
            publishStatus("tcp_valid_frame_probe_runtime_link");
          }
        }
      } else if (now >= tcp_client_handshake_deadline_) {
        forceRuntimeAfterTcp("tcp_handshake_timeout");
        return;
      }
    }

    if (tcp_probe_pending_) {
      if (transport_connected_) {
        tcp_probe_pending_ = false;
        beginEnter();
        publishStatus("tcp_probe_ok_entering_maintenance");
      } else if (now >= tcp_probe_deadline_) {
        forceRuntimeAfterTcp("tcp_probe_timeout");
        return;
      } else if (now >= tcp_probe_next_) {
        sendRuntimeProbe();
        tcp_probe_next_ = now + 150ms;
      }
    }

    if (python_tcp_client_armed_) tcp_pending_rx_.clear();
    forwardTcpPendingRx();
    flushTcpTx();
  }

  void forceRuntimeAfterPython(const std::string &event) {
    const bool was_armed = python_tcp_client_armed_;
    closePythonTcpClient();
    if (!was_armed) {
      publishOwner();
      publishStatus(event + "_unarmed");
      return;
    }
    // If VESC Tool stayed connected while Python had priority, hand maintenance
    // back to it in-place after a zero-current barrier. No runtime command is
    // allowed between the two maintenance owners.
    if (tcp_client_armed_ && maintenance_active_ && transition_ == Transition::NONE) {
      tcp_pending_rx_.clear(); tcp_pending_tx_.clear();
      sendMaintenanceSafeStop();
      publishOwner("VESC_TOOL");
      publishStatus(event + "_resume_vesc_tool");
      return;
    }
    if (transition_ == Transition::ENTER || transition_ == Transition::ENTER_ROUTE) {
      publishMode("RUNTIME"); publishActive(false); transition_ = Transition::NONE;
      publishOwner("RUNTIME");
    } else if (maintenance_active_) {
      sendMaintenanceSafeStop(); transition_ = Transition::EXIT_STOP;
      transition_at_ = std::chrono::steady_clock::now() + 100ms;
    }
    publishStatus(event);
  }

  void queuePythonTx(const std::vector<std::uint8_t> &bytes) {
    if (python_tcp_client_fd_ < 0 || bytes.empty()) return;
    if (python_pending_tx_.size() + bytes.size() > 65536U) {
      forceRuntimeAfterPython("python_tcp_tx_overflow"); return;
    }
    python_pending_tx_.insert(python_pending_tx_.end(), bytes.begin(), bytes.end());
  }

  void flushPythonTx() {
    while (python_tcp_client_fd_ >= 0 && !python_pending_tx_.empty()) {
      const ssize_t n = ::send(python_tcp_client_fd_, python_pending_tx_.data(), python_pending_tx_.size(), MSG_NOSIGNAL);
      if (n > 0) { python_pending_tx_.erase(python_pending_tx_.begin(), python_pending_tx_.begin() + n); continue; }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
      forceRuntimeAfterPython("python_tcp_send_disconnected"); return;
    }
  }

  void forwardPythonPendingRx() {
    if (python_tcp_client_fd_ < 0 || !python_tcp_client_armed_ || !maintenance_active_ ||
        transition_ != Transition::NONE || python_probe_pending_ || python_pending_rx_.empty()) return;
    for (std::size_t budget = 0U; budget < 32U; ++budget) {
      std::vector<std::uint8_t> packet;
      if (!popValidVescFrame(python_pending_rx_, &packet)) break;
      std_msgs::msg::UInt8MultiArray m; m.data = std::move(packet); tx_pub_->publish(m);
    }
  }

  void pythonTcpTick() {
    if (python_tcp_server_fd_ < 0) return;
    if (python_tcp_client_fd_ < 0) {
      sockaddr_in peer{}; socklen_t peer_len = sizeof(peer);
      const int fd = ::accept4(python_tcp_server_fd_, reinterpret_cast<sockaddr *>(&peer), &peer_len,
                               SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd >= 0) {
        int one = 1; (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        python_tcp_client_fd_ = fd;
        python_tcp_client_armed_ = false;
        python_probe_pending_ = false;
        python_pending_rx_.clear(); python_pending_tx_.clear();
        python_client_handshake_deadline_ = std::chrono::steady_clock::now() + 500ms;
        publishOwner();
        publishStatus("python_tcp_connected_waiting_valid_frame");
      }
    }
    if (python_tcp_client_fd_ < 0) return;

    std::uint8_t buf[1024];
    for (;;) {
      const ssize_t n = ::recv(python_tcp_client_fd_, buf, sizeof(buf), 0);
      if (n > 0) {
        if (python_pending_rx_.size() + static_cast<std::size_t>(n) > 65536U) {
          forceRuntimeAfterPython("python_tcp_rx_overflow"); return;
        }
        python_pending_rx_.insert(python_pending_rx_.end(), buf, buf + n);
        continue;
      }
      if (n == 0) { forceRuntimeAfterPython("python_tcp_client_disconnected"); return; }
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      forceRuntimeAfterPython("python_tcp_recv_error"); return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!python_tcp_client_armed_) {
      if (containsValidVescFrame(python_pending_rx_)) {
        if (transition_ == Transition::EXIT_STOP || transition_ == Transition::EXIT_ROUTE) {
          forceRuntimeAfterPython("python_valid_frame_retry_runtime_transition");
          return;
        }
        if (!vehicleIdleForMaintenance()) {
          forceRuntimeAfterPython("python_valid_frame_rejected_vehicle_not_idle");
          return;
        }
        python_tcp_client_armed_ = true;
        if (maintenance_active_) {
          // Highest-priority Python client preempts VESC Tool/API. Drop all
          // lower-priority bytes and put a zero-current barrier in front of it.
          tcp_pending_rx_.clear(); tcp_pending_tx_.clear();
          sendMaintenanceSafeStop();
          publishOwner();
          publishStatus("python_valid_frame_preempted_lower_maintenance");
        } else {
          publishOwner();
          if (transport_connected_) {
            beginEnter();
            publishStatus("python_valid_frame_entering_maintenance");
          } else {
            python_probe_pending_ = true;
            python_probe_deadline_ = now + 1500ms;
            python_probe_next_ = now;
            publishStatus("python_valid_frame_probe_runtime_link");
          }
        }
      } else if (now >= python_client_handshake_deadline_) {
        forceRuntimeAfterPython("python_handshake_timeout");
        return;
      }
    }

    if (python_probe_pending_) {
      if (transport_connected_) {
        python_probe_pending_ = false;
        beginEnter();
        publishStatus("python_probe_ok_entering_maintenance");
      } else if (now >= python_probe_deadline_) {
        forceRuntimeAfterPython("python_tcp_probe_timeout"); return;
      } else if (now >= python_probe_next_) {
        sendRuntimeProbe(); python_probe_next_ = now + 150ms;
      }
    }

    forwardPythonPendingRx();
    flushPythonTx();
  }

  void publishActive(bool value) {
    maintenance_active_ = value; std_msgs::msg::Bool m; m.data = value; active_pub_->publish(m);
  }

  void publishMode(const char *mode) {
    std_msgs::msg::String m; m.data = mode; mode_pub_->publish(m);
  }

  void publishStatus(const std::string &event = "") {
    std_msgs::msg::String m; std::ostringstream o;
    o << "{\"mode\":\"" << (maintenance_active_ ? "maintenance" : "runtime")
      << "\",\"transport_connected\":" << (transport_connected_ ? "true" : "false")
      << ",\"speed_mps\":" << speed_mps_ << ",\"mux\":\"" << mux_source_
      << "\",\"owner\":\"" << maintenanceOwner() << "\""
      << ",\"python_tcp_server\":" << (python_tcp_server_fd_ >= 0 ? "true" : "false")
      << ",\"python_tcp_port\":" << python_tcp_port_ << ",\"python_tcp_client\":" << (python_tcp_client_fd_ >= 0 ? "true" : "false")
      << ",\"python_tcp_armed\":" << (python_tcp_client_armed_ ? "true" : "false")
      << ",\"tcp_server\":" << (tcp_server_fd_ >= 0 ? "true" : "false")
      << ",\"tcp_port\":" << tcp_port_ << ",\"tcp_client\":" << (tcp_client_fd_ >= 0 ? "true" : "false")
      << ",\"tcp_armed\":" << (tcp_client_armed_ ? "true" : "false")
      << ",\"rx_crc_errors\":" << crc_errors_ << ",\"rx_format_errors\":" << format_errors_
      << ",\"last_request_motor\":" << last_request_motor_;
    if (!event.empty()) o << ",\"event\":\"" << event << "\"";
    o << "}"; m.data = o.str(); status_pub_->publish(m);
  }

  bool safeToEnter() const {
    return transport_connected_ && vehicleIdleForMaintenance();
  }

  void beginEnter() {
    if (maintenance_active_ || transition_ != Transition::NONE) return;
    if (!safeToEnter()) { publishStatus("maintenance_rejected_vehicle_not_idle"); return; }
    publishActive(true);  // Ackermann emits its bounded safe-stop while F411 still owns RUNTIME.
    publishOwner();
    transition_ = Transition::ENTER;
    transition_at_ = std::chrono::steady_clock::now() + 150ms;
    publishStatus("maintenance_safe_stop");
  }

  void beginExit() {
    if (!maintenance_active_ || transition_ != Transition::NONE) return;
    closePythonTcpClient();
    closeTcpClient();
    // Stop both motors while F411 still routes MAINTENANCE, then restore RUNTIME.
    sendMaintenanceSafeStop();
    transition_ = Transition::EXIT_STOP;
    transition_at_ = std::chrono::steady_clock::now() + 100ms;
    publishStatus("maintenance_safe_stop_before_runtime");
  }

  void transitionTick() {
    if (transition_ == Transition::NONE || std::chrono::steady_clock::now() < transition_at_) return;
    if (transition_ == Transition::ENTER) {
      publishMode("MAINTENANCE");
      transition_ = Transition::ENTER_ROUTE;
      transition_at_ = std::chrono::steady_clock::now() + 100ms;
      publishStatus("maintenance_route_switch");
    } else if (transition_ == Transition::ENTER_ROUTE) {
      transition_ = Transition::NONE;
      publishStatus("maintenance_active");
    } else if (transition_ == Transition::EXIT_STOP) {
      publishMode("RUNTIME");
      publishStatus("runtime_route_restore");
      transition_ = Transition::EXIT_ROUTE;
      transition_at_ = std::chrono::steady_clock::now() + 150ms;
    } else {
      publishActive(false);
      publishOwner("RUNTIME");
      publishStatus("runtime_active");
      transition_ = Transition::NONE;
    }
  }

  void sendPayload(int motor, std::vector<std::uint8_t> payload) {
    if (!maintenance_active_ || transition_ != Transition::NONE) { publishStatus("command_rejected_not_maintenance"); return; }
    last_request_motor_ = motor;
    const auto packet = frame(motorPayload(motor, std::move(payload)));
    if (packet.empty()) return;
    std_msgs::msg::UInt8MultiArray m; m.data = packet; tx_pub_->publish(m);
  }

  void pollTick() {
    if (!maintenance_active_ || transition_ != Transition::NONE || tcp_client_armed_ || python_tcp_client_armed_) return;
    if (std::chrono::steady_clock::now() < explicit_request_hold_until_) return;
    sendPayload(poll_motor_, {COMM_GET_VALUES});
    poll_motor_ = poll_motor_ == 1 ? 2 : 1;
  }

  void command(const std::string &raw) {
    const auto parts = split(raw, ':');
    if (raw == "MODE:MAINTENANCE") { beginEnter(); return; }
    if (raw == "MODE:RUNTIME" || raw == "MODE:NORMAL") { beginExit(); return; }
    if (!maintenance_active_ || transition_ != Transition::NONE) { publishStatus("command_rejected_not_maintenance"); return; }
    if (python_tcp_client_armed_) { publishStatus("command_rejected_python_has_priority"); return; }
    if (tcp_client_armed_) { publishStatus("command_rejected_tcp_client_owns_maintenance"); return; }

    explicit_request_hold_until_ = std::chrono::steady_clock::now() + 500ms;
    int motor = 0; double value = 0.0;
    if (parts.size() == 2 && parts[0] == "FW" && parseMotor(parts[1], &motor)) { sendPayload(motor, {COMM_FW_VERSION}); return; }
    if (parts.size() == 2 && parts[0] == "VALUES" && parseMotor(parts[1], &motor)) { sendPayload(motor, {COMM_GET_VALUES}); return; }
    if (parts.size() == 3 && parts[0] == "MCCONF" && parseMotor(parts[2], &motor)) {
      if (parts[1] == "GET") sendPayload(motor, {COMM_GET_MCCONF});
      else if (parts[1] == "DEFAULT") sendPayload(motor, {COMM_GET_MCCONF_DEFAULT});
      else publishStatus("bad_mcconf_command");
      return;
    }
    if (parts.size() == 3 && parts[0] == "APPCONF" && parseMotor(parts[2], &motor)) {
      if (parts[1] == "GET") sendPayload(motor, {COMM_GET_APPCONF});
      else if (parts[1] == "DEFAULT") sendPayload(motor, {COMM_GET_APPCONF_DEFAULT});
      else publishStatus("bad_appconf_command");
      return;
    }
    if (parts.size() == 4 && parts[0] == "SET" && parseMotor(parts[2], &motor) && parseDouble(parts[3], &value)) {
      std::uint8_t id = 255U; double scale = 1.0, limit = std::numeric_limits<double>::infinity();
      if (parts[1] == "DUTY") { id = COMM_SET_DUTY; scale = 1e5; limit = max_abs_duty_; }
      else if (parts[1] == "CURRENT") { id = COMM_SET_CURRENT; scale = 1e3; limit = max_abs_current_a_; }
      else if (parts[1] == "BRAKE") { id = COMM_SET_CURRENT_BRAKE; scale = 1e3; limit = max_abs_current_a_; }
      else if (parts[1] == "HANDBRAKE") { id = COMM_SET_HANDBRAKE; scale = 1e3; limit = max_abs_current_a_; }
      else if (parts[1] == "RPM") { id = COMM_SET_RPM; scale = 1.0; limit = max_abs_rpm_; }
      else if (parts[1] == "POS") { id = COMM_SET_POS; scale = 1e6; limit = 360.0; }
      if (id == 255U || std::abs(value) > limit) { publishStatus("setpoint_rejected_limit"); return; }
      std::vector<std::uint8_t> p{id}; appendI32(p, static_cast<std::int32_t>(std::lround(value * scale))); sendPayload(motor, std::move(p)); return;
    }
    if (parts.size() == 4 && parts[0] == "DETECT" && parseMotor(parts[2], &motor) && parseDouble(parts[3], &value)) {
      if (value <= 0.0 || value > std::min(5.0, max_abs_current_a_)) { publishStatus("detect_current_rejected"); return; }
      std::uint8_t id = parts[1] == "ENCODER" ? COMM_DETECT_ENCODER : (parts[1] == "HALL" ? COMM_DETECT_HALL_FOC : 255U);
      if (id == 255U) { publishStatus("bad_detect_command"); return; }
      explicit_request_hold_until_ = std::chrono::steady_clock::now() + 30s;
      std::vector<std::uint8_t> p{id}; appendI32(p, static_cast<std::int32_t>(std::lround(value * 1000.0))); sendPayload(motor, std::move(p)); return;
    }
    if (parts.size() == 2 && parts[0] == "ALIVE" && parseMotor(parts[1], &motor)) { sendPayload(motor, {COMM_ALIVE}); return; }
    if (parts.size() == 2 && parts[0] == "REBOOT" && parseMotor(parts[1], &motor)) { sendPayload(motor, {COMM_REBOOT}); return; }
    if (parts.size() >= 3 && parts[0] == "TERMINAL" && parseMotor(parts[1], &motor)) {
      const auto prefix = std::string("TERMINAL:") + parts[1] + ":";
      const std::string text = raw.substr(prefix.size());
      if (text.empty() || text.size() > 192U) { publishStatus("terminal_command_rejected"); return; }
      std::vector<std::uint8_t> p{COMM_TERMINAL_CMD}; p.insert(p.end(), text.begin(), text.end()); sendPayload(motor, std::move(p)); return;
    }
    if (parts.size() == 3 && parts[0] == "RAW" && parseMotor(parts[1], &motor)) {
      std::vector<std::uint8_t> payload; if (!unhex(parts[2], &payload)) { publishStatus("raw_hex_invalid"); return; }
      sendPayload(motor, std::move(payload)); return;
    }
    publishStatus("unknown_tool_command");
  }

  void consume(const std::vector<std::uint8_t> &bytes) {
    if (python_tcp_client_armed_ && maintenance_active_ && transition_ == Transition::NONE && !python_probe_pending_) {
      queuePythonTx(bytes);
      flushPythonTx();
    } else if (tcp_client_armed_ && maintenance_active_ && transition_ == Transition::NONE && !tcp_probe_pending_) {
      queueTcpTx(bytes);
      flushTcpTx();
    }
    stream_.insert(stream_.end(), bytes.begin(), bytes.end());
    if (stream_.size() > 8192U) stream_.erase(stream_.begin(), stream_.end() - 4096);
    for (;;) {
      while (!stream_.empty() && stream_[0] != 2U && stream_[0] != 3U && stream_[0] != 4U) stream_.erase(stream_.begin());
      if (stream_.size() < 2U) return;
      const auto start = stream_[0]; std::size_t h = 0, n = 0;
      if (start == 2U) { h = 2U; n = stream_[1]; }
      else if (start == 3U) { if (stream_.size() < 3U) return; h = 3U; n = (std::size_t(stream_[1]) << 8U) | stream_[2]; }
      else { if (stream_.size() < 4U) return; h = 4U; n = (std::size_t(stream_[1]) << 16U) | (std::size_t(stream_[2]) << 8U) | stream_[3]; }
      if (!n || n > 4096U) { stream_.erase(stream_.begin()); ++format_errors_; continue; }
      const auto total = h + n + 3U; if (stream_.size() < total) return;
      if (stream_[total - 1U] != 3U) { stream_.erase(stream_.begin()); ++format_errors_; continue; }
      const auto expected = static_cast<std::uint16_t>((std::uint16_t(stream_[h + n]) << 8U) | stream_[h + n + 1U]);
      const auto actual = crc16(stream_.data() + h, n);
      if (expected == actual) handlePayload(std::vector<std::uint8_t>(stream_.begin() + static_cast<std::ptrdiff_t>(h), stream_.begin() + static_cast<std::ptrdiff_t>(h + n)));
      else ++crc_errors_;
      stream_.erase(stream_.begin(), stream_.begin() + static_cast<std::ptrdiff_t>(total));
    }
  }

  void handlePayload(const std::vector<std::uint8_t> &p) {
    if (p.empty()) return;
    std_msgs::msg::String raw; std::ostringstream r;
    r << "{\"motor\":" << last_request_motor_ << ",\"command_id\":" << static_cast<unsigned>(p[0])
      << ",\"payload_hex\":\"" << hex(p) << "\"}"; raw.data = r.str(); raw_pub_->publish(raw);
    if (p[0] == COMM_GET_VALUES && p.size() >= 59U) {
      const unsigned id = p[58]; std_msgs::msg::String m; std::ostringstream o;
      o << "{\"motor\":" << id << ",\"temp_mos_c\":" << double(i16(&p[1])) / 10.0
        << ",\"temp_motor_c\":" << double(i16(&p[3])) / 10.0
        << ",\"current_motor_a\":" << double(i32(&p[5])) / 100.0
        << ",\"current_in_a\":" << double(i32(&p[9])) / 100.0
        << ",\"id_a\":" << double(i32(&p[13])) / 100.0 << ",\"iq_a\":" << double(i32(&p[17])) / 100.0
        << ",\"duty\":" << double(i16(&p[21])) / 1000.0 << ",\"rpm\":" << i32(&p[23])
        << ",\"vbus_v\":" << double(i16(&p[27])) / 10.0 << ",\"fault\":" << static_cast<unsigned>(p[53])
        << ",\"position_deg\":" << double(i32(&p[54])) / 1000000.0 << "}";
      m.data = o.str(); telemetry_pub_->publish(m);
    } else if (p[0] == COMM_FW_VERSION && p.size() >= 4U) {
      std::string hw(reinterpret_cast<const char *>(&p[3]));
      publishStatus(std::string("fw_") + std::to_string(p[1]) + "." + std::to_string(p[2]) + "_" + hw);
    }
  }

  double poll_hz_{10.0}, tcp_service_hz_{1000.0};
  double max_abs_duty_{0.95}, max_abs_current_a_{20.0}, max_abs_rpm_{10000.0};
  bool maintenance_active_{false}, transport_connected_{false}, tcp_enabled_{true}, python_tcp_enabled_{true};
  bool tcp_probe_pending_{false}, python_probe_pending_{false};
  bool tcp_client_armed_{false}, python_tcp_client_armed_{false};
  int tcp_port_{65102}, tcp_server_fd_{-1}, tcp_client_fd_{-1};
  int python_tcp_port_{65101}, python_tcp_server_fd_{-1}, python_tcp_client_fd_{-1};
  double speed_mps_{0.0}; std::string mux_source_;
  Transition transition_{Transition::NONE}; std::chrono::steady_clock::time_point transition_at_{};
  std::chrono::steady_clock::time_point explicit_request_hold_until_{};
  std::chrono::steady_clock::time_point tcp_client_handshake_deadline_{}, python_client_handshake_deadline_{};
  std::chrono::steady_clock::time_point tcp_probe_deadline_{}, tcp_probe_next_{};
  std::chrono::steady_clock::time_point python_probe_deadline_{}, python_probe_next_{};
  int poll_motor_{1}, last_request_motor_{1};
  std::vector<std::uint8_t> stream_, tcp_pending_rx_, tcp_pending_tx_, python_pending_rx_, python_pending_tx_;
  std::uint64_t crc_errors_{0}, format_errors_{0};

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_, runtime_probe_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_, owner_pub_, status_pub_, telemetry_pub_, raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr transport_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mux_sub_, command_sub_;
  rclcpp::TimerBase::SharedPtr transition_timer_, poll_timer_, tcp_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VescToolBridge>());
  rclcpp::shutdown();
  return 0;
}

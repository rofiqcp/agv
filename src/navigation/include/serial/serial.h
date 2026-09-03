#pragma once

// Minimal POSIX serial compatibility layer used by the AGV GNSS/IMU drivers.
// It intentionally implements only the subset of wjwwood/serial that this
// workspace needs, removing the ROS1/catkin serial dependency from ROS 2 Humble.

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace serial
{

class IOException : public std::runtime_error
{
public:
  explicit IOException(const std::string & what) : std::runtime_error(what) {}
};

class SerialException : public std::runtime_error
{
public:
  explicit SerialException(const std::string & what) : std::runtime_error(what) {}
};

struct Timeout
{
  uint32_t read_timeout_constant = 10;
  uint32_t write_timeout_constant = 100;

  static Timeout simpleTimeout(uint32_t timeout_ms)
  {
    Timeout t;
    t.read_timeout_constant = timeout_ms;
    t.write_timeout_constant = timeout_ms > 0 ? timeout_ms : 100;
    return t;
  }
};

class Serial
{
public:
  Serial() = default;
  ~Serial() { closeNoThrow(); }

  Serial(const Serial &) = delete;
  Serial & operator=(const Serial &) = delete;

  void setPort(const std::string & port) { port_ = port; }
  void setBaudrate(uint32_t baudrate) { baudrate_ = baudrate; }
  void setTimeout(const Timeout & timeout) { timeout_ = timeout; }

  bool isOpen() const noexcept { return fd_ >= 0; }

  void open()
  {
    if (isOpen()) return;
    if (port_.empty()) throw SerialException("serial port is empty");

    const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      throw IOException("open(" + port_ + ") failed: " + std::string(std::strerror(errno)));
    }

    fd_ = fd;
    try {
      configureTermios();
      ::tcflush(fd_, TCIFLUSH);
    } catch (...) {
      closeNoThrow();
      throw;
    }
  }

  void close()
  {
    if (!isOpen()) return;
    const int fd = fd_;
    fd_ = -1;
    if (::close(fd) != 0) {
      throw IOException("close(" + port_ + ") failed: " + std::string(std::strerror(errno)));
    }
  }

  void flushInput()
  {
    ensureOpen();
    if (::tcflush(fd_, TCIFLUSH) != 0) {
      throw IOException("tcflush(" + port_ + ") failed: " + std::string(std::strerror(errno)));
    }
  }

  size_t available()
  {
    ensureOpen();
    int bytes = 0;
    if (::ioctl(fd_, FIONREAD, &bytes) != 0) {
      if (errno == EIO || errno == ENODEV || errno == EBADF) {
        throw IOException("ioctl(FIONREAD) failed on " + port_ + ": " + std::string(std::strerror(errno)));
      }
      throw SerialException("ioctl(FIONREAD) failed on " + port_ + ": " + std::string(std::strerror(errno)));
    }
    return bytes > 0 ? static_cast<size_t>(bytes) : 0U;
  }

  std::string read(size_t max_bytes)
  {
    ensureOpen();
    if (max_bytes == 0U) return {};

    struct pollfd pfd {};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int timeout_ms = static_cast<int>(timeout_.read_timeout_constant);
    const int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr == 0) return {};
    if (pr < 0) {
      if (errno == EINTR) return {};
      throw IOException("poll(read) failed on " + port_ + ": " + std::string(std::strerror(errno)));
    }
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw IOException("serial read link lost on " + port_);
    }

    std::string out(max_bytes, '\0');
    const ssize_t n = ::read(fd_, out.data(), max_bytes);
    if (n > 0) {
      out.resize(static_cast<size_t>(n));
      return out;
    }
    if (n == 0) return {};
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return {};
    if (errno == EIO || errno == ENODEV || errno == EBADF) {
      throw IOException("read failed on " + port_ + ": " + std::string(std::strerror(errno)));
    }
    throw SerialException("read failed on " + port_ + ": " + std::string(std::strerror(errno)));
  }

  size_t write(const uint8_t * data, size_t size)
  {
    return writeImpl(reinterpret_cast<const char *>(data), size);
  }

  size_t write(const std::string & data)
  {
    return writeImpl(data.data(), data.size());
  }

  size_t write(const std::vector<uint8_t> & data)
  {
    return write(data.data(), data.size());
  }

private:
  static speed_t baudConstant(uint32_t baud)
  {
    switch (baud) {
      case 1200: return B1200;
      case 2400: return B2400;
      case 4800: return B4800;
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
#ifdef B230400
      case 230400: return B230400;
#endif
#ifdef B460800
      case 460800: return B460800;
#endif
#ifdef B500000
      case 500000: return B500000;
#endif
#ifdef B576000
      case 576000: return B576000;
#endif
#ifdef B921600
      case 921600: return B921600;
#endif
      default:
        throw SerialException("unsupported baudrate: " + std::to_string(baud));
    }
  }

  void configureTermios()
  {
    struct termios tio {};
    if (::tcgetattr(fd_, &tio) != 0) {
      throw IOException("tcgetattr(" + port_ + ") failed: " + std::string(std::strerror(errno)));
    }

    ::cfmakeraw(&tio);
    tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tio.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tio.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tio.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tio.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    const speed_t speed = baudConstant(baudrate_);
    if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) {
      throw IOException("cfset*speed(" + port_ + ") failed: " + std::string(std::strerror(errno)));
    }
    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
      throw IOException("tcsetattr(" + port_ + ") failed: " + std::string(std::strerror(errno)));
    }
  }

  size_t writeImpl(const char * data, size_t size)
  {
    ensureOpen();
    size_t done = 0;
    while (done < size) {
      const ssize_t n = ::write(fd_, data + done, size - done);
      if (n > 0) {
        done += static_cast<size_t>(n);
        continue;
      }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        struct pollfd pfd {};
        pfd.fd = fd_;
        pfd.events = POLLOUT;
        const int pr = ::poll(&pfd, 1, static_cast<int>(timeout_.write_timeout_constant));
        if (pr > 0) continue;
        if (pr == 0) throw SerialException("serial write timeout on " + port_);
        if (errno == EINTR) continue;
        throw IOException("poll(write) failed on " + port_ + ": " + std::string(std::strerror(errno)));
      }
      if (errno == EIO || errno == ENODEV || errno == EBADF) {
        throw IOException("write failed on " + port_ + ": " + std::string(std::strerror(errno)));
      }
      throw SerialException("write failed on " + port_ + ": " + std::string(std::strerror(errno)));
    }
    return done;
  }

  void ensureOpen() const
  {
    if (!isOpen()) throw SerialException("serial port is not open");
  }

  void closeNoThrow() noexcept
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  std::string port_;
  uint32_t baudrate_ = 9600;
  Timeout timeout_{};
  int fd_ = -1;
};

}  // namespace serial

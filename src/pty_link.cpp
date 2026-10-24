#include "rabbits_sim/pty_link.hpp"

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>

namespace rabbits_sim
{

PtyLink::~PtyLink()
{
  Close();
}

bool PtyLink::Open(const std::string & link_path)
{
  if (is_open()) {
    return false;
  }
  char name[128] = {};
  if (openpty(&master_fd_, &slave_fd_, name, nullptr, nullptr) != 0) {
    master_fd_ = -1;
    slave_fd_ = -1;
    return false;
  }
  slave_path_ = name;

  struct termios tio;
  if (tcgetattr(slave_fd_, &tio) == 0) {
    cfmakeraw(&tio);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(slave_fd_, TCSANOW, &tio);
  }
  fcntl(master_fd_, F_SETFL, fcntl(master_fd_, F_GETFL, 0) | O_NONBLOCK);

  unlink(link_path.c_str());
  if (symlink(slave_path_.c_str(), link_path.c_str()) != 0) {
    Close();
    return false;
  }
  link_path_ = link_path;
  return true;
}

void PtyLink::Close()
{
  if (!link_path_.empty()) {
    unlink(link_path_.c_str());
    link_path_.clear();
  }
  if (slave_fd_ >= 0) {
    close(slave_fd_);
    slave_fd_ = -1;
  }
  if (master_fd_ >= 0) {
    close(master_fd_);
    master_fd_ = -1;
  }
  slave_path_.clear();
}

int PtyLink::Read(uint8_t * buf, size_t len)
{
  if (!is_open()) {
    return -1;
  }
  const ssize_t n = read(master_fd_, buf, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EIO || errno == EINTR) {
      return 0;
    }
    return -1;
  }
  return static_cast<int>(n);
}

int PtyLink::Write(const uint8_t * buf, size_t len)
{
  if (!is_open()) {
    return -1;
  }
  const ssize_t n = write(master_fd_, buf, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EIO || errno == EINTR) {
      return 0;
    }
    return -1;
  }
  return static_cast<int>(n);
}

}  // namespace rabbits_sim

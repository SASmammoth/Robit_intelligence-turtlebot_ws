#include "tb_uart/ubuntu_uart.hpp"

#include <asm/termbits.h>   // termios2, BOTHER (<termios.h>와 함께 include 금지)
#include <sys/ioctl.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <string>

namespace tb {

UbuntuUart::~UbuntuUart()
{
  for (auto &kv : fds_) close(kv.second);
}

bool UbuntuUart::uart_init(int channel, unsigned int baudrate)
{
  auto it = fds_.find(channel);
  if (it != fds_.end()) {
    close(it->second);
    fds_.erase(it);
  }

  const std::string dev = "/dev/ttyUSB" + std::to_string(channel);
  int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    std::perror("uart_init: open");
    return false;
  }

  struct termios2 tio;
  if (ioctl(fd, TCGETS2, &tio) != 0) {
    std::perror("uart_init: TCGETS2");
    close(fd);
    return false;
  }

  tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL | IXON | IXOFF | IXANY);
  tio.c_oflag &= ~OPOST;
  tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | CBAUD);
  tio.c_cflag |= (CS8 | CLOCAL | CREAD | BOTHER);
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (ioctl(fd, TCSETS2, &tio) != 0) {
    std::perror("uart_init: TCSETS2");
    close(fd);
    return false;
  }

  ioctl(fd, TCFLSH, TCIOFLUSH);
  fds_[channel] = fd;
  return true;
}

int UbuntuUart::transmit(int channel, const uint8_t *packet, int length)
{
  auto it = fds_.find(channel);
  if (it == fds_.end() || packet == nullptr || length < 0) return -1;
  const int fd = it->second;

  int sent = 0;
  while (sent < length) {
    ssize_t n = write(fd, &packet[sent], 1);
    if (n == 1) {
      ++sent;
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      struct pollfd pfd = {fd, POLLOUT, 0};
      poll(&pfd, 1, 100);
      continue;
    }
    std::perror("transmit: write");
    return -1;
  }
  ioctl(fd, TCSBRK, 1);   // tcdrain
  return sent;
}

int UbuntuUart::receive(int channel, uint8_t *buffer, int length, int timeout_ms)
{
  auto it = fds_.find(channel);
  if (it == fds_.end() || buffer == nullptr || length <= 0) return -1;
  const int fd = it->second;

  struct pollfd pfd = {fd, POLLIN, 0};
  int ret = poll(&pfd, 1, timeout_ms);
  if (ret < 0) return (errno == EINTR) ? 0 : -1;
  if (ret == 0) return 0;
  if (pfd.revents & (POLLERR | POLLNVAL | POLLHUP)) return -1;   // 장치 분리

  ssize_t n = read(fd, buffer, length);
  if (n > 0) return static_cast<int>(n);
  if (n < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
  return -1;   // 장치 분리 또는 read 오류
}

void UbuntuUart::uart_close(int channel)
{
  auto it = fds_.find(channel);
  if (it == fds_.end()) return;
  close(it->second);
  fds_.erase(it);
}

}  // namespace tb
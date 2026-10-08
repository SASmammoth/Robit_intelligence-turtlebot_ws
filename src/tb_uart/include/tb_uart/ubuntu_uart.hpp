#ifndef TB_UART__UBUNTU_UART_HPP_
#define TB_UART__UBUNTU_UART_HPP_

#include <cstdint>
#include <map>

namespace tb {

class UbuntuUart {
public:
  UbuntuUart() = default;
  ~UbuntuUart();
  UbuntuUart(const UbuntuUart &) = delete;
  UbuntuUart &operator=(const UbuntuUart &) = delete;

  // /dev/ttyUSB<channel> 을 열고 baud rate 설정 (8N1, raw, 흐름제어 없음)
  bool uart_init(int channel, unsigned int baudrate);

  // 1바이트 단위로 length만큼 송신. 송신한 바이트 수, 오류 시 -1
  int transmit(int channel, const uint8_t *packet, int length);

  // 데이터가 도착하면 즉시 반환 (최대 length 바이트).
  // 타임아웃이면 0, 오류/장치 분리 시 -1
  int receive(int channel, uint8_t *buffer, int length, int timeout_ms);

  // 포트 닫기
  void uart_close(int channel);

private:
  std::map<int, int> fds_;   // channel -> fd
};

}  // namespace tb

#endif  // TB_UART__UBUNTU_UART_HPP_
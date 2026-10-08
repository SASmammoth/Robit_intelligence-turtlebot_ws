#!/bin/bash
# setup_uart.sh : 새 PC에서 UART 접근 권한 설정
echo 'SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/60-tb-uart.rules
sudo udevadm control --reload-rules
sudo usermod -aG dialout $USER
echo "어댑터를 다시 꽂으세요. dialout 그룹은 재로그인 후 적용됩니다."
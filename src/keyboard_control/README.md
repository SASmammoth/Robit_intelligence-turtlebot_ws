# keyboard_control

터미널의 w/a/s/d/q 키 입력을 ROS2 topic 명령으로 변환해 발행하는 패키지입니다.
시작 시 `start` 핸드셰이크를 거친 뒤에만 키 입력을 처리합니다.

- ROS2 Jazzy, `ament_python`
- 노드: `keyboard_node`
- 메시지 타입: `std_msgs/String`

## 실행

```bash
cd <워크스페이스>
colcon build --packages-select keyboard_control
source install/setup.bash
ros2 run keyboard_control keyboard_node
```

노드는 터미널의 stdin을 직접 읽습니다. 반드시 터미널에서 `ros2 run`으로 실행해야 하며,
`ros2 launch`처럼 stdin이 연결되지 않는 방식으로는 키 입력을 받을 수 없습니다.

## Topic

`keyboard_node.py` 상단에 상수로 선언되어 있습니다. topic 이름을 바꾸려면 이 두 줄만 수정하면 됩니다.

```python
TX_TOPIC = 'TB_Uart_RX'   # 발행 (노드 -> 로봇)
RX_TOPIC = 'TB_Uart_TX'   # 수신 (로봇 -> 노드)
```

### TX_TOPIC (발행)

| 메시지 | 발행 시점 |
|---|---|
| `start` | 노드 시작 직후. `start`를 수신할 때까지 1초마다 재발행 |
| `direction 1 -1` | `RX_TOPIC`에서 `start`를 수신한 직후 (1회) |
| `velocity 50 50` | `w` 입력 (전진) |
| `velocity 0 50` | `a` 입력 |
| `velocity -50 -50` | `s` 입력 (후진) |
| `velocity 50 0` | `d` 입력 |
| `velocity 0 0` | 키가 눌리지 않은 상태로 바뀔 때 (정지) |
| `quit` | `q` 입력. 발행 후 노드 종료 |

`velocity` 명령은 키보드 입력 상태가 **바뀔 때만** 발행합니다. 같은 키를 계속 누르고 있어도 반복 발행하지 않습니다.

### RX_TOPIC (수신)

| 메시지 | 동작 |
|---|---|
| `start` | 핸드셰이크 완료. `direction 1 -1` 발행 후 키 입력 처리 시작 |

`start` 외의 메시지는 로그만 출력하고 무시합니다. 키 입력 처리가 시작된 뒤에는 수신 메시지로 상태가 바뀌지 않습니다.

## 실행 구조

```
노드 시작
  │  TX: "start"
  ▼
[대기 상태]  ── 1초마다 "start" 재발행
  │           (이 동안 키 입력은 모두 무시)
  │  RX: "start" 수신
  ▼
TX: "direction 1 -1"
  ▼
[동작 상태]  ── w/a/s/d 입력 상태가 바뀔 때마다 "velocity ..." 발행
  │           (start 직후 "velocity 0 0"은 발행하지 않음)
  │  q 입력
  ▼
TX: "quit" → 터미널 설정 복원 → 노드 종료
```

메인 루프는 `rclpy.spin_once()`(5ms 주기)로 수신 콜백을 처리하고,
이어서 `poll_keyboard()`로 stdin을 확인합니다.

## 조작 방법

| 키 | 동작 | 발행 |
|---|---|---|
| `w` | 전진 | `velocity 50 50` |
| `a` | 좌측 | `velocity 0 50` |
| `s` | 후진 | `velocity -50 -50` |
| `d` | 우측 | `velocity 50 0` |
| (없음) | 정지 | `velocity 0 0` |
| `q` | 종료 | `quit` |

- 대소문자는 구분하지 않습니다.
- 노드를 실행한 뒤 `start`를 수신할 때까지는 어떤 키도 동작하지 않습니다.
  (`q`도 무시됩니다. 강제 종료는 `Ctrl+C`)
- 다른 키로 바꾸면 바로 새 명령이 발행됩니다.

### 키를 뗀 것으로 판단하는 방식

터미널에는 키를 뗐다는 이벤트가 없습니다. 그래서 일정 시간 동안 같은 키 입력이 없으면
키를 뗀 것으로 간주하고 `velocity 0 0`을 발행합니다.

| 상수 | 값 | 의미 |
|---|---|---|
| `INITIAL_TIMEOUT` | 0.6초 | 키 반복이 시작되기 전. OS의 반복 시작 지연보다 길어야 함 |
| `REPEAT_TIMEOUT` | 0.1초 | 키 반복이 확인된 뒤 |
| `REPEAT_GAP_MAX` | 0.12초 | 같은 키 입력 간격이 이보다 짧으면 반복 중으로 판단 |
| `LOOP_PERIOD` | 0.005초 | 메인 루프 확인 주기 |

- 길게 누르고 있다가 떼면 약 0.1초 안에 정지합니다.
- 짧게 톡 치면 키 반복이 시작되지 않아 약 0.6초 뒤에 정지합니다.
- 누르고 있는데 중간에 한 번 멈춘다면 `INITIAL_TIMEOUT`을 키보드의 반복 시작 지연보다 길게 늘리세요.

## 패키지 구조

```
keyboard_control/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── keyboard_control
└── keyboard_control/
    ├── __init__.py
    └── keyboard_node.py
```

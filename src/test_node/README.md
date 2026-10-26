# test_node

ROS 2 Jazzy용 터미널 입출력 테스트용 패키지이다. publisher, subscriber 노드 2개로 구성

| 노드 | 파일 | 역할 |
|------|------|------|
| `pub_node` | `test_node/pub_node.py` | 터미널에 입력한 값을 `topic`으로 publish |
| `sub_node` | `test_node/sub_node.py` | `topic`을 subscribe하여 받은 값을 터미널에 출력 |

토픽과 메시지 타입은 실행 시 파라미터로 지정 (다시 빌드할 필요 없음)

| 파라미터 | pub_node 기본값 | sub_node 기본값 |
|----------|-----------------|-----------------|
| `topic` | `TB_Uart_RX` | `TB_Uart_TX` |
| `type` | `std_msgs/msg/String` | `std_msgs/msg/String` |

- `type`이 String이면 입력한 텍스트가 그대로 `data`가 됨
- 그 외 타입은 한 줄 YAML로 입력. 예) `{data: true}`, `{detections: [{class_id: 7}]}`

## 요구 사항

- Ubuntu 24.04
- ROS 2 Jazzy
- colcon, git

## 1. 워크스페이스에 설치

[main/README.md](https://github.com/mck737338/Robit_intelligence_turtle-bot/blob/main/REAMDE.md) 의 3-1 참고

## 2. 토픽 지정

```bash
ros2 run test_node pub_node --ros-args -p topic:=/brain/run
ros2 run test_node sub_node --ros-args -p topic:=/mission/pause -p type:=std_msgs/msg/Bool
```

같은 노드를 여러 개 띄울 때는 `-r __node:=이름`으로 노드 이름을 다르게 지정

## 3. 빌드

```bash
cd ~/${WS_NAME}
colcon build --packages-select test_node
source install/setup.bash
```

## 4. 실행

터미널을 두 개 열고 각각 `source install/setup.bash`를 한 뒤 실행

```bash
# 터미널 1: 입력한 텍스트를 tx_topic으로 publish (Enter로 전송)
ros2 run test_node pub_node

# 터미널 2: rx_topic으로 들어온 값을 출력
ros2 run test_node sub_node
```


## 5. 동작 확인

```bash
# pub_node 입력이 tx_topic으로 나오는지 확인
ros2 topic echo /TB_Uart_RX

# rx_topic으로 값을 보내 sub_node 출력 확인
ros2 topic pub --once /TB_Uart_TX std_msgs/msg/String "{data: 'hello'}"
```

토픽 이름을 바꿨다면 위 명령의 토픽 이름도 같이 변경

## 파일 구조

```
test_node/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── test_node        # 빈 파일 (삭제 금지, git에 반드시 포함)
└── test_node/
    ├── __init__.py
    ├── pub_node.py
    └── sub_node.py
```

## 문제 해결

- `can't copy '.../resource/test_node'` 에러: `resource/test_node` 빈 파일이 없는 경우. `touch resource/test_node` 후 `build/test_node`, `install/test_node`를 지우고 다시 빌드하세요.
- `Package 'test_node' not found`: 빌드 실패 또는 `source install/setup.bash`를 하지 않은 경우입니다.

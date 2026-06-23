# ROS 2 매니퓰레이터 학습 가이드

이 문서는 `manipulator` 패키지를 따라가며 URDF, TF, ros2_control, Gazebo, 하드웨어 인터페이스가 어떻게 연결되는지 설명한다.

## 1. 패키지 전체 그림

이 패키지의 핵심은 형상과 제어 백엔드를 분리한 것이다.

```text
                         manipulator.xacro
                    링크, 조인트, mesh, joint limit
                               │
              ┌────────────────┼────────────────┐
              │                │                │
        display.launch    sim URDF          real URDF
              │                │                │
 joint_state_publisher   GazeboSystem    ArduinoHardwareInterface
              │                │                │
             RViz       Gazebo physics       serial servo
```

RViz, Gazebo, 실제 로봇이 같은 링크와 조인트 이름을 공유한다. 달라지는 것은 “명령을 실제로 수행하는 대상”이다.

## 2. URDF와 xacro

URDF는 로봇의 강체인 `link`와 link 사이의 운동 관계인 `joint`를 XML로 표현한다.

xacro는 반복되는 URDF를 매크로와 변수로 작성할 수 있게 해준다. 이 패키지에서는 `urdf/manipulator.xacro`가 공통 모델이다.

### 링크와 조인트 수

- 형상 링크: `link_1` ~ `link_7`, 7개
- 기준 링크: `world`, 1개
- 고정 조인트: `world_joint`, 1개
- 가동 조인트: 회전 4개 + 그리퍼 2개, 6개

따라서 URDF 트리 전체에는 링크 8개와 조인트 7개가 있다.

```text
world
└── world_joint
    └── link_1
        └── joint_1 → link_2
            └── joint_2 → link_3
                └── joint_3 → link_4
                    └── joint_4 → link_5
                        ├── joint_5_left  → link_6
                        └── joint_5_right → link_7
```

### mimic 그리퍼

`joint_5_right`는 다음 선언으로 왼쪽을 따라간다.

```xml
<mimic joint="joint_5_left" multiplier="1" offset="0"/>
```

두 조인트 값은 같지만 축이 각각 `0 1 0`, `0 -1 0`이므로 실제 이동 방향은 반대다. 그래서 하나의 명령으로 두 손가락이 대칭 이동한다.

가동 조인트는 6개지만 독립 명령축은 5개인 이유가 이것이다.

## 3. TF와 RViz 경로

`robot_state_publisher`는 두 입력을 사용한다.

- `robot_description`: 전체 URDF
- `/joint_states`: 현재 조인트 위치

이 둘을 조합해 `/tf`와 `/tf_static`을 발행한다.

```text
joint_state_publisher_gui ── /joint_states ──┐
                                             ▼
URDF ─────────────────────────── robot_state_publisher ── /tf
                                                              │
                                                              ▼
                                                             RViz
```

`display.launch.py`에는 controller_manager가 없다. 슬라이더가 가짜 조인트 상태를 만들어 형상과 TF만 확인한다.

오른쪽 그리퍼는 mimic이므로 GUI에는 독립 슬라이더가 나타나지 않는다.

## 4. ros2_control

ros2_control은 컨트롤러와 하드웨어 사이의 공통 규격이다.

```text
JointTrajectory
      │
      ▼
joint_trajectory_controller
      │ position command interface
      ▼
hardware interface
      │
      ├── Gazebo joint
      └── Arduino serial
```

컨트롤러는 하드웨어가 Gazebo인지 Arduino인지 알 필요가 없다. 동일한 `position` command interface에 목표값을 쓴다.

### 이 패키지의 인터페이스

`my_controllers.yaml`은 다음 5축을 제어한다.

```text
joint_1
joint_2
joint_3
joint_4
joint_5_left
```

명령과 상태 인터페이스는 모두 `position`이다.

`joint_5_right`는 독립 controller joint가 아니다. Gazebo에서는 mimic 파라미터가 왼쪽 상태를 오른쪽에 적용하고, 실물에서는 하나의 그리퍼 서보가 양쪽 기구를 움직인다고 가정한다.

## 5. Gazebo 경로

`manipulator_sim.urdf.xacro`는 공통 모델에 다음 두 요소를 더한다.

1. `gazebo_ros2_control/GazeboSystem`
2. controller YAML을 읽는 `libgazebo_ros2_control.so`

Gazebo 플러그인이 controller_manager를 생성하고 시뮬레이션 조인트를 hardware interface처럼 제공한다.

`gazebo.launch.py`의 순서는 다음과 같다.

```text
Gazebo 시작
robot_state_publisher 시작
spawn_entity로 로봇 생성
spawn 완료
joint_state_broadcaster 시작
joint_trajectory_controller 시작
```

spawn 전에 컨트롤러를 시작하면 `/controller_manager`가 아직 없어 실패하거나 불필요하게 기다릴 수 있으므로 이벤트 순서를 명시했다.

## 6. 실제 Arduino 경로

`manipulator_real.urdf.xacro`는 `ArduinoHardwareInterface`를 지정한다.

### 생명주기

| 함수 | 역할 |
|---|---|
| `on_init()` | 파라미터와 인터페이스 검증, 초기 위치와 서보 매핑 준비 |
| `export_state_interfaces()` | 5개 position 상태 제공 |
| `export_command_interfaces()` | 5개 position 명령 제공 |
| `on_activate()` | 시리얼 포트 열기, 현재 상태에서 명령 시작 |
| `read()` | 마지막 명령을 현재 위치로 보고 |
| `write()` | 명령을 서보 각도로 변환해 CSV 전송 |
| `on_deactivate()` | 시리얼 포트 닫기 |

시리얼 포트를 `on_init()`이 아니라 `on_activate()`에서 여는 이유는 하드웨어 생명주기와 실제 장치 연결 시점을 맞추기 위해서다.

### 위치에서 서보 각도로 변환

각 조인트는 다음 네 값을 가진다.

```text
command_min, command_max
servo_min_angle, servo_max_angle
```

변환식은 선형 보간이다.

```text
ratio = (command - command_min) / (command_max - command_min)
servo = servo_min_angle + ratio × (servo_max_angle - servo_min_angle)
```

이 방식은 회전 조인트뿐 아니라 미터 단위인 prismatic 그리퍼에도 동일하게 적용된다.

`joint_3`처럼 서보 방향이 반대인 경우 `servo_min_angle=180`, `servo_max_angle=0`으로 설정한다.

### 시리얼 드라이버

`ArduinoSerialDriver`는 Linux `termios`를 사용한다.

- 8 data bits
- parity 없음
- 1 stop bit
- hardware flow control 없음
- raw mode

한 명령은 다음 CSV 한 줄이다.

```text
90,90,90,90,0\n
```

Arduino 쪽에서는 줄바꿈을 기준으로 한 프레임을 읽어야 한다.

### 100 Hz 제어와 20 Hz 시리얼

controller_manager는 100 Hz로 동작하지만 hobby servo와 문자열 시리얼 통신은 그렇게 빠른 갱신이 필요하지 않다.

하드웨어 인터페이스는 `write_rate_hz`를 사용해 실제 시리얼 전송을 기본 20 Hz로 제한한다. controller의 내부 계산과 state publication은 계속 100 Hz로 유지된다.

## 7. 안전 관련 판단

### 초기값

명령 벡터를 NaN으로 시작하면 첫 제어 주기에 잘못된 정수 변환이 일어날 수 있다. 현재 코드는 URDF의 `initial_value`로 상태와 명령을 함께 초기화한다.

### 범위 제한

controller에서 잘못된 값이 들어와도 hardware interface가 URDF 범위로 한 번 더 제한한다.

### open-loop의 의미

서보에서 실제 각도를 읽지 않으므로 다음 등식은 가정일 뿐이다.

```text
마지막 명령 위치 = 현재 위치
```

서보가 걸리거나 전원이 꺼져도 ROS에는 명령 위치에 도달한 것처럼 보인다. 충돌 감지나 정밀 제어가 필요하다면 엔코더 피드백과 별도 오류 처리가 필요하다.

## 8. 실행하며 확인할 항목

### RViz

```bash
ros2 launch manipulator display.launch.py
```

- Fixed Frame이 `world`인지
- 5개 슬라이더가 나타나는지
- 왼쪽 그리퍼 조작 시 오른쪽도 대칭 이동하는지

### Gazebo

```bash
ros2 launch manipulator gazebo.launch.py
ros2 control list_controllers
ros2 topic echo /joint_states
```

- 두 컨트롤러가 active인지
- trajectory 명령 후 5개 명령축이 목표값으로 이동하는지
- `joint_5_right`가 `joint_5_left`를 따라가는지

### 실제 로봇

```bash
ros2 launch manipulator real_robot.launch.py serial_port:=/dev/ttyACM0
```

- 실행 사용자가 시리얼 장치 권한을 갖는지
- Arduino 보드레이트가 launch 값과 같은지
- 서보를 기구물에 연결하기 전에 최소·중립·최대 방향이 맞는지
- 그리퍼 0 ~ 8 mm가 실제 서보의 안전 범위와 맞는지

## 9. 남아 있는 공학적 과제

패키지 코드 경로는 완성했지만 실제 로봇 성능을 위해서는 물리 정보가 더 필요하다.

- CAD 또는 측정을 기반으로 질량과 관성 텐서 보정
- 실제 서보별 각도 한계와 중립 위치 캘리브레이션
- Arduino 펌웨어의 CSV 파서와 watchdog 구현
- 통신 단절 시 서보 정지/토크 해제 정책
- 엔코더를 사용할 경우 closed-loop state feedback 추가
- 장기적으로 Gazebo Classic에서 modern Gazebo로 전환

이 항목들은 소프트웨어 누락이라기보다 실제 기구와 전장 사양을 알아야 결정할 수 있는 부분이다.

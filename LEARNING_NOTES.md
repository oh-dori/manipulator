# ROS 2 매니퓰레이터 학습 노트

이 노트는 `manipulator` 패키지를 처음 공부하는 사람을 위해, 파일을 하나씩 순서대로 따라가며 URDF, TF, ros2_control, Gazebo, 하드웨어 인터페이스가 어떻게 연결되는지 설명한다.

## 들어가기 — 이 패키지가 하는 일과 학습 순서

### 패키지 전체 그림

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

### 학습 순서 지도

아래 순서로 보면, 가장 어려운 하드웨어 인터페이스(4번)에 도착했을 때 이미 필요한 사전 지식이 모두 깔려 있다.

| 순서 | 파일 | 역할 |
|---|---|---|
| 1 | [urdf/manipulator.xacro](urdf/manipulator.xacro) | 로봇이 **어떻게 생겼나** (링크·조인트·메쉬) — 모든 것의 기반 |
| 2 | [launch/display.launch.py](launch/display.launch.py) | 그 모델을 **RViz로 띄우는** 가장 단순한 실행 |
| 3 | [urdf/manipulator_real.urdf.xacro](urdf/manipulator_real.urdf.xacro) + [config/my_controllers.yaml](config/my_controllers.yaml) | 모델에 **실제 로봇 제어 설정**을 붙임 |
| 4 | [src/arduino_hardware_interface.cpp](src/arduino_hardware_interface.cpp) | 제어 명령을 **실제로 처리**하는 C++ (직접 만든 하드웨어 인터페이스) |
| 5 | [src/arduino_serial_driver.cpp](src/arduino_serial_driver.cpp) | 아두이노로 **시리얼 전송** |

Gazebo 경로([launch/gazebo.launch.py](launch/gazebo.launch.py), [urdf/manipulator_sim.urdf.xacro](urdf/manipulator_sim.urdf.xacro))는 실제 로봇 경로를 이해한 뒤 보면 쉬우므로 곁가지로 뺐다.

---

## 1. 로봇 형상 — manipulator.xacro

이 파일은 로봇의 형상(생김새)을 글로 적어둔 설계도다. 움직이는 코드는 없다.

> **xacro**: URDF를 변수와 반복 틀로 편하게 쓰게 해주는 언어. 실행할 때 순수 URDF로 펼쳐진다.
> **URDF**: “이 로봇은 이런 부품들이 이렇게 연결돼 있다”를 XML로 적은 것.

### URDF의 부품은 두 종류 — link와 joint

`link`는 로봇의 뼈대 한 마디(강체 부품)다. 가장 단순한 link는 이렇다.

```xml
<link name="world" />
```

`joint`는 두 link를 잇는 관절이다. 항상 부모 link → 자식 link를 잇고, `type`으로 움직임 방식을 정한다.

```xml
<joint name="world_joint" type="fixed">
  <parent link="world"/>            <!-- 부모 뼈대 -->
  <child link="link_1"/>            <!-- 자식 뼈대 -->
  <origin xyz="0 0 0" rpy="0 0 0"/> <!-- 부모로부터 떨어진 위치 -->
</joint>
```

joint의 `type`은 세 가지다.

- `fixed`: 안 움직이는 고정 연결
- `revolute`: 회전 관절
- `prismatic`: 직선으로 미끄러지는 관절 (그리퍼처럼 벌어졌다 닫힘)

### 반복을 줄이는 매크로

link 하나를 제대로 정의하려면 보이는 모양(`visual`), 충돌 영역(`collision`), 질량(`inertial`)을 모두 적어야 해서 길어진다. 그것을 매번 반복하지 않으려고 **매크로**(반복 XML을 재사용하는 틀)를 쓴다. 예를 들어 link 일곱 개는 다음 한 줄짜리 호출로 만든다.

```xml
<xacro:create_link link_name="link_1" mass="0.01" origin_xyz="0.04 0.00 0.00" gazebo_material="Grey"/>
```

이 파일의 매크로는 세 개다.

- `create_link`: 뼈대 하나를 찍어낸다.
- `create_revolute_joint`: 회전 관절을 찍어낸다.
- `ros2_control_joint`: 제어용 설정을 찍어낸다. 이 파일에서는 쓰지 않고, 뒤의 Gazebo 시뮬레이션 파일에서 쓴다.

즉 파일 위쪽 절반(매크로 정의)은 “도장”이고, 아래쪽 절반(로봇 조립)은 그 도장을 찍은 결과다.

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

---

## 2. 가장 단순한 실행 — display.launch.py

1번에서는 로봇의 설계도(URDF) 파일만 봤다. 이제 그 설계도를 ROS에서 **실제로 실행**해, 모델이 제대로 생겼는지 화면(RViz)에서 눈으로 확인한다. 로봇을 움직이지는 않는, 가장 단순한 실행 경로다.

`display.launch.py`는 노드 세 개를 띄운다. 아래에서 하나씩, 데이터가 흐르는 순서대로 본다.

### ① joint_state_publisher_gui — 가짜 조인트 값 만들기

슬라이더 GUI 노드다. 사용자가 슬라이더를 움직이면 "지금 각 조인트가 이 값이다"를 **/joint_states** 토픽으로 발행한다.

> **/joint_states**: 각 조인트가 지금 몇 도(회전)·몇 미터(직선)인지를 담는 토픽. 메시지 안에 조인트 이름과 현재 위치값이 들어 있다.

실제 로봇이 없으므로, 이 노드가 가짜 조인트 값을 만들어 주는 역할이다.

### ② robot_state_publisher — 조인트 값을 부품 위치로 변환

핵심 변환기다. 두 입력을 받는다.

- **robot_description**: 1번에서 만든 URDF(로봇 설계도) 전체를 담은 값. 노드들이 "이 로봇이 어떻게 생겼는지" 알아야 하므로 공통으로 참조한다.
- **/joint_states**: 바로 위 ①번에서 만든 현재 조인트 위치.

이 둘을 조합해 "각 부품(link)이 공간의 어디에, 어떤 방향으로 있는가"를 계산한다. 이 위치·방향 정보를 ROS에서는 **TF(좌표 변환)**라고 부른다.

> **프레임과 TF**: 각 부품(link)은 자기만의 좌표계인 **프레임(frame)**을 가진다(`world`, `link_1`, `link_2` …). **TF**는 "어떤 프레임이 다른 프레임에 대해 어디에, 어떤 방향으로 있는가"를 나타내는 정보다. 조인트 각도(예: "joint_2는 30도")만으로는 화면에 그릴 수 없고, 그것을 각 부품의 실제 위치·방향으로 바꿔야 하는데, 그 변환이 바로 TF다.

robot_state_publisher는 이 TF를 두 토픽으로 발행한다. 고정 조인트(`world_joint`)처럼 안 변하는 변환은 한 번만 보내면 되므로 `/tf_static`으로, 움직이는 조인트의 변환은 계속 바뀌므로 `/tf`로 보낸다.

> **의문 — 왜 TF는 부모 프레임 기준 상대 변환으로 저장할까?** (모든 링크를 베이스 좌표 기준으로 바로 저장하면 안 되나?)
> ① 각 변환은 조인트 하나에만 묶이므로, 조인트가 움직여도 그와 닿은 한 칸만 갱신하면 된다(변경 국소화) → ② 그래서 좌표계 트리의 각 칸을 서로 다른 노드가 나눠서 발행할 수 있다. ③ 베이스(또는 임의의 두 프레임) 기준 좌표는 TF가 트리를 곱해 필요할 때 계산해 주므로 잃는 것이 없다 — 그 곱셈은 베이스 기준으로 저장하더라도 갱신 때마다 어차피 치러야 하는 비용이다.

### ③ rviz2 — 화면에 그리기

화면 노드다. `robot_description`으로 로봇의 생김새를, TF로 각 부품의 위치를 받아 3D로 그린다.

### 데이터 흐름

위 세 노드를 한 그림으로 모으면 이렇다.

```text
joint_state_publisher_gui ── /joint_states ──┐
                                             ▼
URDF(robot_description) ───────── robot_state_publisher ── /tf
                                                              │
                                                              ▼
                                                             RViz
```

읽는 법: 왼쪽의 가짜 조인트 값(①)과 위쪽의 로봇 형상이 robot_state_publisher(②)로 들어가 TF로 바뀌고, RViz(③)가 그 TF를 받아 로봇을 그린다.

### 이 경로의 특징

- `display.launch.py`에는 **controller_manager가 없다.** (controller_manager는 실제 제어를 담당하는 부품으로, 4번에서 설명한다.) 즉 이 경로에는 실제 제어가 전혀 없고, 슬라이더가 만든 가짜 조인트 상태로 형상과 TF만 확인한다.
- 오른쪽 그리퍼(`joint_5_right`)는 mimic이므로 GUI에 독립 슬라이더가 나타나지 않는다. 왼쪽 그리퍼를 따라 움직인다.

---

## 3. 실제 로봇 제어 설정 — manipulator_real.urdf.xacro + my_controllers.yaml

앞의 `manipulator.xacro`는 형상만 정의한다. 실제 로봇을 제어하려면 두 가지 설정을 더 붙인다.

첫째, `manipulator_real.urdf.xacro`는 공통 모델에 `<ros2_control>` 블록을 더해, 어떤 하드웨어 플러그인(`ArduinoHardwareInterface`)을 쓸지와 각 조인트의 명령 범위·서보 보정값을 선언한다.

둘째, `my_controllers.yaml`은 어떤 컨트롤러를 띄울지 정한다. 다음 5축을 제어한다.

```text
joint_1
joint_2
joint_3
joint_4
joint_5_left
```

명령과 상태 인터페이스는 모두 `position`이다.

`joint_5_right`는 독립 controller joint가 아니다. Gazebo에서는 mimic 파라미터가 왼쪽 상태를 오른쪽에 적용하고, 실물에서는 하나의 그리퍼 서보가 양쪽 기구를 움직인다고 가정한다.

---

## 4. 하드웨어 인터페이스 (직접 만든 C++) — arduino_hardware_interface.cpp

### ros2_control 핵심 개념

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

### 100 Hz 제어와 20 Hz 시리얼

controller_manager는 100 Hz로 동작하지만 hobby servo와 문자열 시리얼 통신은 그렇게 빠른 갱신이 필요하지 않다.

하드웨어 인터페이스는 `write_rate_hz`를 사용해 실제 시리얼 전송을 기본 20 Hz로 제한한다. controller의 내부 계산과 state publication은 계속 100 Hz로 유지된다.

---

## 5. 시리얼 드라이버 — arduino_serial_driver.cpp

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

---

## 곁가지. Gazebo 경로 — manipulator_sim.urdf.xacro + gazebo.launch.py

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

---

## 부록 A. 안전 설계 판단

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

---

## 부록 B. 현재 한계와 남은 과제

패키지 코드 경로는 완성했지만 실제 로봇 성능을 위해서는 물리 정보가 더 필요하다.

- CAD 또는 측정을 기반으로 질량과 관성 텐서 보정
- 실제 서보별 각도 한계와 중립 위치 캘리브레이션
- Arduino 펌웨어의 CSV 파서와 watchdog 구현
- 통신 단절 시 서보 정지/토크 해제 정책
- 엔코더를 사용할 경우 closed-loop state feedback 추가
- 장기적으로 Gazebo Classic에서 modern Gazebo로 전환

이 항목들은 소프트웨어 누락이라기보다 실제 기구와 전장 사양을 알아야 결정할 수 있는 부분이다.

---

## 부록 C. 실행·검증 명령 모음

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

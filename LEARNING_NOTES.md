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

2번의 `display.launch.py`에서는 로봇을 실제로 움직인 것이 아니다. 사용자가 GUI 슬라이더를 움직이면 가짜 `/joint_states`가 만들어지고, `robot_state_publisher`가 그 값을 TF로 바꿔 RViz에 보여줬다.

```text
joint_state_publisher_gui
  -> /joint_states
  -> robot_state_publisher
  -> /tf
  -> RViz
```

이 흐름에는 Arduino가 없다. 서보로 명령을 보내는 부분도 없고, 시리얼 포트를 여는 부분도 없다.

실제 로봇에서는 중간에 이런 부품이 하나 더 필요하다.

```text
ROS 2 controller
  -> 조인트 위치 명령
  -> ArduinoHardwareInterface
  -> 서보 각도 CSV
  -> Arduino
```

`ArduinoHardwareInterface`는 이 프로젝트에서 만든 C++ 중간 어댑터다. ROS 2 쪽에서는 `joint_1 = 0.5 rad` 같은 조인트 위치 명령을 받고, Arduino 쪽으로는 `90,120,45,90,30` 같은 서보 각도 CSV를 보낸다.

3장은 이 중간 어댑터를 ROS 2 제어 흐름에 끼워 넣기 위한 설정을 읽는 구간이다.

```text
manipulator_real.urdf.xacro
  "이 로봇은 ArduinoHardwareInterface로 제어한다"는 정보를 URDF에 추가한다.

my_controllers.yaml
  "어떤 controller를 실행해서 명령과 상태를 처리할지" 정한다.
```

여기서 controller는 ROS 2에서 조인트 명령과 상태를 처리하는 부품이다. 하드웨어에 position 명령을 쓰는 controller도 있고, 하드웨어 상태를 `/joint_states`로 내보내는 controller도 있다.

### ① manipulator_real.urdf.xacro — 하드웨어 연결 설명서

`manipulator.xacro`는 로봇의 모양과 관절 구조만 설명한다.

```text
link
joint
mesh
mimic
```

그래서 실제 로봇용 파일인 `manipulator_real.urdf.xacro`는 이 기본 모델을 먼저 가져온다.

```xml
<xacro:include filename="$(find manipulator)/urdf/manipulator.xacro" />
```

이제 로봇의 모양은 준비됐다. 하지만 아직 ROS 2는 다음을 모른다.

```text
Arduino와 어떻게 연결하지?
어떤 C++ 하드웨어 코드를 써야 하지?
어떤 joint를 실제로 제어하지?
조인트 명령 범위와 서보 각도 범위는 어떻게 맞추지?
```

이 질문에 답하는 부분이 `<ros2_control>` 블록이다.

```xml
<ros2_control name="RealRobot" type="system">
  ...
</ros2_control>
```

`type="system"`은 여러 조인트를 하나의 하드웨어 장치처럼 다룬다는 뜻이다. 이 프로젝트에서는 Arduino 하나가 여러 서보를 함께 제어하므로 system으로 둔다.

먼저 어떤 하드웨어 어댑터를 쓸지 적는다.

```xml
<hardware>
  <plugin>arduino_hardware_interface/ArduinoHardwareInterface</plugin>
  <param name="serial_port">$(arg serial_port)</param>
  <param name="baud_rate">$(arg baud_rate)</param>
  <param name="write_rate_hz">$(arg write_rate_hz)</param>
  <param name="dry_run">$(arg dry_run)</param>
</hardware>
```

`plugin` 줄은 "Arduino와 통신할 때 `ArduinoHardwareInterface`를 사용한다"는 뜻이다. 이 이름을 보고 ros2_control이 C++ 플러그인을 불러온다.

아래 값들은 Arduino 연결과 테스트 모드 설정이다.

```text
serial_port
  Arduino가 잡힌 장치 경로. 예: /dev/ttyUSB0, /dev/ttyACM0

baud_rate
  Arduino와 맞출 시리얼 통신 속도. 현재 arm.ino 기준 기본값은 9600

write_rate_hz
  Arduino로 명령을 최대 몇 Hz로 보낼지 정하는 값

dry_run
  true이면 시리얼 포트를 열지 않고, Arduino로 보낼 CSV 값만 로그로 출력한다.
```

이 값들은 컴퓨터나 연결 상태에 따라 바뀔 수 있으므로 xacro 인자로 빼두었다. 아래 줄들은 "이 xacro 파일은 `serial_port`, `baud_rate`, `write_rate_hz`, `dry_run`이라는 값을 받을 수 있다"는 선언이다. 실제 실행에서는 `real_robot.launch.py`가 launch 인자를 받고, 그 값을 xacro 명령에 넘겨 완성된 URDF를 만든다.

```xml
<xacro:arg name="serial_port" default="/dev/ttyUSB0"/>
<xacro:arg name="baud_rate" default="9600"/>
<xacro:arg name="write_rate_hz" default="20"/>
<xacro:arg name="dry_run" default="false"/>
```

그래서 실행할 때 포트나 dry-run 여부를 바꿔 줄 수 있다. launch에서 받은 값이 xacro로 들어가 `robot_description`에 포함되는 흐름은 아래의 `③ real_robot.launch.py에서 묶이는 방식`에서 다시 본다.

```bash
ros2 launch manipulator real_robot.launch.py serial_port:=/dev/ttyACM0
```

Arduino 없이 안전하게 흐름만 확인하려면 dry-run을 켠다.

```bash
ros2 launch manipulator real_robot.launch.py dry_run:=true
```

다음으로 실제 제어할 joint를 적는다. 예를 들어 `joint_1`은 이렇게 설정되어 있다.

```xml
<joint name="joint_1">
  <param name="servo_min_angle">0</param>
  <param name="servo_max_angle">180</param>
  <command_interface name="position">
    <param name="min">${-pi/2}</param>
    <param name="max">${pi/2}</param>
  </command_interface>
  <state_interface name="position">
    <param name="initial_value">0</param>
  </state_interface>
</joint>
```

이 블록은 세 가지를 말한다.

```text
1. joint_1은 position 명령으로 제어한다.
2. joint_1의 현재 상태도 position으로 제공한다.
3. ROS 쪽 조인트 범위와 실제 서보 각도 범위를 연결한다.
```

`command_interface`는 controller가 하드웨어에 쓰는 명령 통로다. 여기서는 전부 `position`이다.

`state_interface`는 하드웨어가 ROS 쪽에 알려주는 상태 통로다. 이것도 여기서는 `position`이다.

`min`, `max`는 ROS 쪽 조인트 값의 범위다. 회전 조인트는 radian 단위이고, 그리퍼처럼 직선 이동하는 prismatic 조인트는 meter 단위다.

`servo_min_angle`, `servo_max_angle`은 Arduino 서보에 보낼 실제 각도 범위다. 이 값은 표준 URDF 값이 아니라 `ArduinoHardwareInterface`가 읽는 커스텀 파라미터다.

서보 장착 방향이 ROS joint 방향과 반대라면 아래처럼 두 값을 서로 바꿔서 보정할 수 있다.

```xml
<param name="servo_min_angle">180</param>
<param name="servo_max_angle">0</param>
```

이 경우 ROS 쪽 조인트 명령이 커질수록 Arduino로 보내는 서보 각도는 작아진다.

현재 `joint_3`에는 이 반전 설정을 사용하지 않는다. ROS 명령 범위 `-pi ~ 0`을 서보 각도 `0 ~ 180`에 대응시킨다.

```text
ROS -pi rad -> Arduino 0도
ROS   0 rad -> Arduino 180도
```

따라서 `joint_3`의 ROS 초기값 `0`은 Arduino 코드의 초기 서보 각도 `180도`와 일치한다.

그리퍼인 `joint_5_left`는 회전 조인트가 아니라 직선 이동 조인트다.

```xml
<command_interface name="position">
  <param name="min">0</param>
  <param name="max">0.008</param>
</command_interface>
```

`0.008`은 8 mm다. `ArduinoHardwareInterface`는 이 0~0.008 m 값을 다시 30~90도 서보 각도로 바꿔 Arduino에 보낸다.

`joint_5_right`는 `<ros2_control>` 블록에 없다. 실물에서는 하나의 그리퍼 서보가 양쪽을 같이 움직인다고 보고, 모델에서는 `joint_5_right`가 `joint_5_left`를 mimic한다.

### ② my_controllers.yaml — controller 실행 목록

`manipulator_real.urdf.xacro`까지 읽으면 하드웨어 어댑터와 제어할 joint는 알 수 있다. 하지만 아직 "ROS 2 안에서 누가 명령을 받고, 누가 상태를 내보낼지"는 정하지 않았다.

그 설정이 `my_controllers.yaml`이다.

먼저 ros2_control의 실행 구조를 구분해서 보자.

```text
controller_manager 패키지
  ROS 2가 제공하는 패키지 이름

ros2_control_node
  controller_manager 패키지 안에 있는 실행 노드
  real_robot.launch.py가 이 노드를 실행한다.

controller
  ros2_control_node 안에서 로드되어 동작하는 제어 부품
  이 프로젝트에서는 아래 두 개를 쓴다.
```

```text
joint_state_broadcaster
  상태 담당 controller
  하드웨어 상태를 /joint_states로 내보낸다.

joint_trajectory_controller
  명령 담당 controller
  목표 궤적을 받아 조인트 position 명령을 만든다.
```

즉 여기서 controller는 독립적으로 launch되는 노드라기보다, `ros2_control_node` 안에 들어가서 동작하는 제어 모듈에 가깝다.

이제 YAML을 읽어보자. 먼저 `ros2_control_node`의 갱신 주기를 정한다.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
```

`update_rate: 100`은 초당 100번 정도 다음 흐름을 반복한다는 뜻이다.

```text
하드웨어 상태 읽기
-> controller 계산
-> 하드웨어 명령 쓰기
```

그 아래에는 `ros2_control_node` 안에 로드할 controller 종류를 등록한다.

```yaml
joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  type: joint_trajectory_controller/JointTrajectoryController
```

여기서 `type`은 어떤 controller 플러그인을 불러올지 적는 이름이다. 직접 만든 클래스 이름이 아니라, ROS 2 controller 패키지들이 제공하는 등록 이름이다.

둘의 역할은 다음처럼 다르다.

```text
joint_state_broadcaster
  하드웨어의 state_interface를 읽어 /joint_states를 발행한다.
  실제 로봇 실행에서는 2번의 joint_state_publisher_gui 대신 이 controller가 상태를 내보낸다.

joint_trajectory_controller
  목표 궤적 명령을 받아 각 joint의 command_interface에 position 명령을 쓴다.
```

마지막으로 `joint_trajectory_controller`가 어떤 joint를 제어할지 적는다.

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5_left
    command_interfaces:
      - position
    state_interfaces:
      - position
```

이 목록은 `manipulator_real.urdf.xacro`의 `<ros2_control>` 블록에 있는 joint 이름과 맞아야 한다.

```text
URDF의 <ros2_control>에 joint_1이 있음
YAML의 joints 목록에도 joint_1이 있음
-> controller가 joint_1을 제어할 수 있음

YAML에는 joint_5_right가 없음
-> controller가 직접 제어하지 않음
```

### ③ real_robot.launch.py에서 묶이는 방식

`manipulator_real.urdf.xacro`와 `my_controllers.yaml`은 파일만 있어서는 실행되지 않는다. launch 파일이 둘을 노드에 전달한다.

`real_robot.launch.py`는 먼저 xacro를 실행해 실제 URDF 문자열을 만든다. 이때 시리얼 포트 같은 실행 인자도 함께 넘긴다.

```text
manipulator_real.urdf.xacro
  + serial_port
  + baud_rate
  + write_rate_hz
  + dry_run
  -> 완성된 URDF 문자열
  -> robot_description 파라미터
```

그 다음 같은 `robot_description`을 두 노드에 넣는다.

```text
robot_state_publisher 노드
  robot_description을 읽고 link/joint 구조로 TF를 발행한다.

ros2_control_node 노드
  robot_description 안의 <ros2_control> 블록을 읽는다.
  my_controllers.yaml도 함께 읽는다.
  ArduinoHardwareInterface와 controller들을 연결한다.

rviz2 노드
  /tf와 robot_description을 받아 실제 제어 흐름에서 로봇 모델이 어떻게 움직이는지 보여준다.
```

최종 흐름은 다음과 같다.

```text
외부의 JointTrajectory 명령
  -> joint_trajectory_controller
  -> 5개 position command interface
  -> ArduinoHardwareInterface::write()
  -> radian/meter 명령을 서보 각도로 변환
  -> Arduino로 CSV 전송

ArduinoHardwareInterface::read()
  -> 5개 position state interface
  -> joint_state_broadcaster
  -> /joint_states
  -> robot_state_publisher
  -> /tf, /tf_static
  -> RViz
```

---

## 4. 하드웨어 인터페이스 (직접 만든 C++) — arduino_hardware_interface.cpp

### ros2_control 핵심 개념

3장 마지막에서 터미널로 보낸 조인트 명령은 `trajectory_msgs/msg/JointTrajectory` 형식의 메시지다. 이 메시지 하나에 움직일 조인트와 목표 위치, 이동 시간을 함께 담는다.

```text
joint_names
  명령을 받을 조인트 이름

points.positions
  각 조인트가 도달할 목표 위치

points.time_from_start
  그 위치까지 이동할 시간
```

이 메시지를 `/joint_trajectory_controller/joint_trajectory` 토픽에 발행하면 `joint_trajectory_controller`가 받는다.

`joint_trajectory_controller`는 `controller_manager` 패키지에서 제공하는 controller 기능 모듈이며, 실행할 때 `ros2_control_node` 안에 불러온다. 이 controller는 메시지에 적힌 목표 시간에 맞춰 중간 위치들을 계산하고, 현재 시점의 목표 위치를 각 조인트의 `position command interface`에 기록한다.

`position command interface`는 controller가 계산한 위치를 저장하고 하드웨어 쪽에 전달하는 내부 메모리 통로다. 토픽을 하나 더 발행하는 것이 아니라, `ros2_control_node` 안에서 값을 공유한다.

이 값을 하드웨어 쪽에서 읽는 코드가 우리가 작성한 `ArduinoHardwareInterface` C++ 클래스다. 이 클래스도 실행할 때 `ros2_control_node` 안에 불러오며, ros2_control에서는 이처럼 controller와 실제 장치 사이를 연결하는 클래스를 `hardware interface`라고 부른다.

실제 로봇 실행에서는 `ArduinoHardwareInterface::write()`가 command interface의 radian 또는 meter 값을 읽고, 서보 각도로 변환해 CSV를 만든다.

```text
JointTrajectory 메시지
  조인트 이름, 목표 위치, 이동 시간을 담아 토픽으로 전송
      ↓
joint_trajectory_controller
  ros2_control_node 안에서 실행되는 controller 기능 모듈
  시간에 따른 각 조인트의 목표 위치를 계산
      ↓
position command interface
  계산된 위치 명령을 저장하는 내부 메모리 통로
      ↓
ArduinoHardwareInterface::write()
  우리가 작성한 C++ 하드웨어 인터페이스 기능 모듈
  ROS 위치 명령을 서보 각도로 변환하고 CSV 생성
      ↓
Arduino serial
  CSV 문자열을 실제 Arduino에 전송
```

Gazebo를 실행할 때는 `ArduinoHardwareInterface` 대신 Gazebo용 하드웨어 인터페이스가 같은 자리에 들어간다. 둘이 동시에 사용되는 것이 아니라, 실제 로봇용 xacro와 시뮬레이션용 xacro 중 무엇을 실행했는지에 따라 하나가 선택된다.

`joint_trajectory_controller`는 어느 구현이 선택되었는지 알 필요가 없다. 두 구현 모두 ros2_control의 공통 규격인 `position command interface`를 제공하기 때문이다.

### ArduinoHardwareInterface가 실행되는 순서

`ArduinoHardwareInterface`가 플러그인으로 불러와지면 `ros2_control_node` 내부의 controller manager가 이 클래스의 함수들을 호출한다. 실행 과정은 처음 한 번 수행하는 준비 단계와 계속 반복하는 제어 단계로 나뉜다.

#### 1. 처음 한 번 수행하는 준비

먼저 `on_init()`이 `robot_description`의 `<ros2_control>` 블록에서 시리얼 포트, 통신 속도, dry-run 설정과 각 조인트의 범위를 읽는다.

이때 조인트 수에 맞춰 두 메모리 배열도 만든다.

```text
hw_commands_
  controller가 계산한 위치 명령을 저장

hw_positions_
  현재 조인트 위치라고 보고할 값을 저장
```

두 배열은 xacro의 `initial_value`로 초기화된다. 아직 Arduino에 연결하거나 명령을 보내지는 않는다.

다음으로 두 export 함수가 controller와 이 배열을 연결한다.

```text
export_command_interfaces()
  각 조인트의 position command interface가
  hw_commands_[index]를 가리키도록 연결

export_state_interfaces()
  각 조인트의 position state interface가
  hw_positions_[index]를 가리키도록 연결
```

여기서 export는 값을 밖으로 전송한다는 뜻이 아니다. controller가 해당 메모리를 읽거나 쓸 수 있도록 주소를 공개한다는 뜻이다. 이 연결도 시작할 때 한 번 만들며, 제어 주기마다 다시 만드는 것이 아니다.

준비가 끝나면 `on_activate()`가 호출된다. 시작 순간에 갑자기 다른 명령이 나가지 않도록 `hw_commands_`를 현재 `hw_positions_`와 같게 맞춘다. 실제 실행이면 시리얼 포트를 열고, dry-run이면 포트를 열지 않은 채 활성화된다.

#### 2. 활성화된 동안 반복하는 제어

활성화된 뒤에는 controller manager가 기본 100 Hz로 다음 순서를 반복한다.

```text
ArduinoHardwareInterface::read()
        ↓
controller들의 update
        ↓
ArduinoHardwareInterface::write()
```

현재 서보에서는 실제 각도를 돌려받지 못한다. 따라서 `read()`는 이전 주기에 명령했던 값을 현재 위치라고 가정한다.

```cpp
hw_positions_ = hw_commands_;
```

이 대입으로 `hw_positions_`가 바뀌면 연결된 position state interface에서도 같은 값이 보인다. `joint_state_broadcaster`는 이 값을 읽어 `/joint_states`를 발행하고, `joint_trajectory_controller`도 현재 상태로 읽을 수 있다.

그 다음 `joint_trajectory_controller`가 이번 주기의 목표 위치를 계산해 position command interface에 기록한다. 이 interface는 `hw_commands_`에 연결되어 있으므로 별도의 복사 함수 없이 배열 값이 바로 바뀐다.

마지막으로 `write()`가 새 `hw_commands_`를 읽는다. 각 ROS 위치를 서보 각도로 변환하고 CSV 한 줄을 만든 뒤 Arduino에 전송한다. dry-run에서는 같은 CSV를 시리얼로 보내지 않고 터미널에 출력한다.

```text
read()
  이전 hw_commands_를 hw_positions_에 복사

joint_state_broadcaster
  hw_positions_를 읽어 /joint_states 발행

joint_trajectory_controller
  새 목표 위치를 hw_commands_에 기록

write()
  hw_commands_를 서보 각도와 CSV로 변환해 전송
```

실행을 종료하거나 하드웨어가 비활성화되면 `on_deactivate()`가 시리얼 포트를 닫는다. dry-run에서는 처음부터 포트를 열지 않았으므로 닫을 포트도 없다.

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

서보 방향이 반대인 경우에는 `servo_min_angle`과 `servo_max_angle`을 서로 바꿔서 보정한다.

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
90,0,180,90,90\n
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

- Arduino 없이 테스트할 때는 `dry_run:=true`로 실행했는지
- dry-run에서 시리얼 포트를 열지 않고 Arduino로 보낼 CSV만 출력하는지
- RViz가 함께 뜨고 `/joint_states` → `/tf` 흐름으로 모델이 움직이는지
- 실제 Arduino에 보낼 때는 기본값인 `dry_run:=false`로 실행하는지
- 실행 사용자가 시리얼 장치 권한을 갖는지
- Arduino 보드레이트가 launch 값과 같은지
- 서보를 기구물에 연결하기 전에 최소·중립·최대 방향이 맞는지
- 그리퍼 0 ~ 8 mm가 실제 서보의 안전 범위와 맞는지

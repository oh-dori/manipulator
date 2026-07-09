# ROS 2 매니퓰레이터 학습 노트

이 노트는 `manipulator` 리포지토리 안의 `my_manipulator` 패키지를 처음 공부하는 사람을 위해, 파일을 하나씩 순서대로 따라가며 URDF, TF, ros2_control, Gazebo, 하드웨어 인터페이스가 어떻게 연결되는지 설명한다. MoveIt 2 설정은 같은 리포의 별도 패키지 `my_manipulator_moveit`에서 관리한다.

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
| 1 | [urdf/manipulator.xacro](my_manipulator/urdf/manipulator.xacro) | 로봇이 **어떻게 생겼나** (링크·조인트·메쉬) — 모든 것의 기반 |
| 2 | [launch/display.launch.py](my_manipulator/launch/display.launch.py) | 그 모델을 **RViz로 띄우는** 가장 단순한 실행 |
| 3 | [urdf/manipulator_real.urdf.xacro](my_manipulator/urdf/manipulator_real.urdf.xacro) + [config/my_controllers.yaml](my_manipulator/config/my_controllers.yaml) | 모델에 **실제 로봇 제어 설정**을 붙임 |
| 4 | [src/arduino_hardware_interface.cpp](my_manipulator/src/arduino_hardware_interface.cpp) | 제어 명령을 **실제로 처리**하는 C++ (직접 만든 하드웨어 인터페이스) |
| 5 | [src/arduino_serial_driver.cpp](my_manipulator/src/arduino_serial_driver.cpp) | 아두이노로 **시리얼 전송** |
| 6 | [urdf/manipulator_sim.urdf.xacro](my_manipulator/urdf/manipulator_sim.urdf.xacro) + [launch/gazebo.launch.py](my_manipulator/launch/gazebo.launch.py) | 실제 하드웨어를 **Gazebo 물리 시뮬레이션으로 교체** |
| 7 | `my_manipulator_moveit` (추가 예정) | 목표 자세에서 충돌 없는 조인트 궤적을 계산해 기존 controller로 전달 |

6장에서는 1~5장에서 배운 모델, controller, command/state interface가 Gazebo에서도 어떻게 그대로 이어지는지 비교한다. 7장에서는 그 위에 MoveIt 2를 올려, 조인트 값을 직접 정하던 단계에서 엔드이펙터의 목표 자세와 충돌 회피 경로를 다루는 단계로 넘어간다.

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

`ArduinoHardwareInterface`는 이 프로젝트에서 만든 C++ 중간 어댑터다. ROS 2 쪽에서는 `joint_1 = 0.5 rad` 같은 조인트 위치 명령을 받고, Arduino 쪽으로는 `90,120,45,90,30,1` 같은 서보 각도와 즉시 실행 여부를 CSV로 보낸다.

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
<xacro:include filename="$(find my_manipulator)/urdf/manipulator.xacro" />
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
ros2 launch my_manipulator real_robot.launch.py serial_port:=/dev/ttyACM0
```

Arduino 없이 안전하게 흐름만 확인하려면 dry-run을 켠다.

```bash
ros2 launch my_manipulator real_robot.launch.py dry_run:=true
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

서보 장착 방향이 ROS joint 방향과 반대라면 서보 각도 범위는 실제 안전 각도 그대로 두고, `command_interface`의 `min`/`max`를 서로 바꿔서 보정한다.

```xml
<param name="servo_min_angle">0</param>
<param name="servo_max_angle">180</param>
<command_interface name="position">
  <param name="min">${pi/2}</param>
  <param name="max">${-pi/2}</param>
</command_interface>
```

이 경우 ROS 쪽 조인트 명령이 작아질수록 Arduino로 보내는 서보 각도는 커진다.

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

arm_controller
  명령 담당 controller 인스턴스
  joint_trajectory_controller plugin으로 만들어진다.
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

arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
```

여기서 `type`은 어떤 controller 플러그인을 불러올지 적는 이름이다. 직접 만든 클래스 이름이 아니라, ROS 2 controller 패키지들이 제공하는 등록 이름이다.

둘의 역할은 다음처럼 다르다.

```text
joint_state_broadcaster
  하드웨어의 state_interface를 읽어 /joint_states를 발행한다.
  실제 로봇 실행에서는 2번의 joint_state_publisher_gui 대신 이 controller가 상태를 내보낸다.

arm_controller
  joint_trajectory_controller plugin으로 만들어지는 controller 인스턴스다.
  목표 궤적 명령을 받아 각 joint의 command_interface에 position 명령을 쓴다.
```

마지막으로 `arm_controller`가 어떤 joint를 제어할지 적는다.

```yaml
arm_controller:
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
```

최종 흐름은 다음과 같다.

```text
외부의 JointTrajectory 명령
  -> arm_controller
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
```

이 흐름을 화면으로 보고 싶을 때만 RViz를 별도 명령으로 붙인다. `display.launch.py`처럼 트랙바가 있는 확인용 launch를 같이 쓰지 않는다.

```bash
rviz2 -d ~/ros2_ws/src/manipulator/my_manipulator/rviz/display.rviz
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

이 메시지를 `/arm_controller/joint_trajectory` 토픽에 발행하면 `arm_controller`가 받는다.

`arm_controller`는 `controller_manager` 패키지에서 제공하는 controller 기능 모듈이며, 실행할 때 `ros2_control_node` 안에 불러온다. 이 controller는 메시지에 적힌 목표 시간에 맞춰 중간 위치들을 계산하고, 현재 시점의 목표 위치를 각 조인트의 `position command interface`에 기록한다.

`position command interface`는 controller가 계산한 위치를 저장하고 하드웨어 쪽에 전달하는 내부 메모리 통로다. 토픽을 하나 더 발행하는 것이 아니라, `ros2_control_node` 안에서 값을 공유한다.

이 값을 하드웨어 쪽에서 읽는 코드가 우리가 작성한 `ArduinoHardwareInterface` C++ 클래스다. 이 클래스도 실행할 때 `ros2_control_node` 안에 불러오며, ros2_control에서는 이처럼 controller와 실제 장치 사이를 연결하는 클래스를 `hardware interface`라고 부른다.

실제 로봇 실행에서는 `ArduinoHardwareInterface::write()`가 command interface의 radian 또는 meter 값을 읽고, 서보 각도로 변환해 CSV를 만든다.

```text
JointTrajectory 메시지
  조인트 이름, 목표 위치, 이동 시간을 담아 토픽으로 전송
      ↓
arm_controller
  ros2_control_node 안에서 joint_trajectory_controller plugin으로 실행되는 controller 인스턴스
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

`arm_controller`는 어느 구현이 선택되었는지 알 필요가 없다. 두 구현 모두 ros2_control의 공통 규격인 `position command interface`를 제공하기 때문이다.

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

이 대입으로 `hw_positions_`가 바뀌면 연결된 position state interface에서도 같은 값이 보인다. `joint_state_broadcaster`는 이 값을 읽어 `/joint_states`를 발행하고, `arm_controller`도 현재 상태로 읽을 수 있다.

그 다음 `arm_controller`가 이번 주기의 목표 위치를 계산해 position command interface에 기록한다. 이 interface는 `hw_commands_`에 연결되어 있으므로 별도의 복사 함수 없이 배열 값이 바로 바뀐다.

마지막으로 `write()`가 새 `hw_commands_`를 읽는다. 각 ROS 위치를 서보 각도로 변환하고 CSV 한 줄을 만든 뒤 Arduino에 전송한다. dry-run에서는 같은 CSV를 시리얼로 보내지 않고 터미널에 출력한다.

```text
read()
  이전 hw_commands_를 hw_positions_에 복사

joint_state_broadcaster
  hw_positions_를 읽어 /joint_states 발행

arm_controller
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

서보 방향이 반대인 경우에는 서보 각도 범위 대신 `command_min`과 `command_max`를 서로 바꿔서 보정한다. 이 프로젝트의 `ArduinoHardwareInterface`는 내림차순 command range도 허용하고, clamp할 때는 두 값 중 작은 값과 큰 값을 안전 범위로 사용한다.

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
90,0,180,90,90,1\n
```

앞의 5개 값은 서보 각도이고, 마지막 값은 실행 방식이다. `1`이면 즉시
적용하고 `0`이면 Arduino가 정해진 시간 동안 목표 각도로 이동한다.
Arduino 쪽에서는 줄바꿈을 기준으로 한 프레임을 읽어야 한다.

---

## 6. Gazebo 시뮬레이션 — manipulator_sim.urdf.xacro + gazebo.launch.py

### 왜 실제 로봇을 움직인 다음 Gazebo를 배우는가

지금까지 세 가지 단계를 직접 확인했다.

```text
display.launch.py
  가짜 joint_states로 모델과 TF를 확인

real_robot.launch.py dry_run
  controller 계산과 Arduino용 CSV를 확인

real_robot.launch.py
  시리얼을 통해 실제 서보까지 명령 전달
```

이제 Gazebo를 사용하는 이유는 단순히 화면에 로봇을 하나 더 띄우기 위해서가 아니다. 실제 로봇 경로에서 하드웨어 부분만 물리 시뮬레이터로 교체했을 때 같은 ROS 2 controller가 그대로 동작하는지 확인하기 위해서다.

세 실행 환경은 다음처럼 역할이 다르다.

| 환경 | 조인트 상태를 만드는 주체 | 물리 계산 | 주된 확인 대상 |
|---|---|---|---|
| RViz + GUI | 사용자가 움직인 슬라이더 | 없음 | 형상, joint 축, TF |
| 실제 로봇 | 현재는 마지막 명령값을 상태로 가정 | 실제 기구에서 발생 | 시리얼, 서보 방향, 실제 동작 |
| Gazebo | 물리 엔진이 계산한 조인트 위치 | 있음 | 충돌, 질량, 관성, 중력, 동적 움직임 |

RViz는 전달받은 상태를 그대로 그린다. 목표 위치가 물리적으로 가능한지, 링크끼리 충돌하는지, 중력 때문에 처지는지는 판단하지 않는다. Gazebo는 명령을 받은 뒤 물리 법칙을 적용해 다음 상태를 계산한다.

따라서 이 장의 학습 목표는 다음 세 가지다.

```text
1. controller와 hardware가 어떻게 분리되는지 확인한다.
2. 명령 위치와 물리적으로 계산된 실제 위치의 차이를 이해한다.
3. visual, collision, inertial 정보가 각각 어디에 사용되는지 확인한다.
```

### 핵심 구조 — controller는 유지하고 hardware만 교체한다

실제 로봇의 제어 흐름은 다음과 같다.

```text
trajectory_msgs/msg/JointTrajectory 메시지
  -> arm_controller (joint_trajectory_controller plugin으로 만든 controller)
  -> position command interface (ros2_control 내부에서 위치 명령을 공유하는 메모리)
  -> ArduinoHardwareInterface::write() (직접 작성한 C++ hardware interface 클래스의 함수)
  -> ArduinoSerialDriver (CSV를 USB serial로 전송하는 C++ 통신 클래스)
  -> Arduino 펌웨어와 서보 (명령을 실행하는 실제 하드웨어)
```

Gazebo에서는 controller 앞부분을 바꾸지 않는다.

```text
trajectory_msgs/msg/JointTrajectory 메시지
  -> arm_controller (joint_trajectory_controller plugin으로 만든 controller)
  -> position command interface (ros2_control 내부에서 위치 명령을 공유하는 메모리)
  -> GazeboSystem (Gazebo용 hardware interface 플러그인)
  -> Gazebo 조인트와 물리 엔진 (명령을 적용하고 상태를 계산하는 시뮬레이션)
```

`arm_controller`는 상대가 실제 Arduino인지 Gazebo인지 알 필요가 없다. 두 hardware implementation이 모두 같은 position command/state interface를 제공하기 때문이다.

이 교체 가능성이 ros2_control을 사용하는 중요한 이유다. 상위 제어 코드는 유지하면서 실제 장치, dry-run 구현, 시뮬레이터를 선택할 수 있다.

### ① manipulator_sim.urdf.xacro — 시뮬레이션용 hardware 설정

시뮬레이션 파일도 실제 로봇 파일처럼 공통 모델부터 가져온다.

```xml
<xacro:include filename="$(find my_manipulator)/urdf/manipulator.xacro" />
```

따라서 RViz, 실제 로봇, Gazebo가 같은 link, joint, mesh와 joint limit을 사용한다. 달라지는 것은 `<ros2_control>`의 hardware plugin이다.

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  ...
</ros2_control>
```

실제 로봇에서는 이 자리에 직접 만든 plugin이 있었다.

```xml
<plugin>arduino_hardware_interface/ArduinoHardwareInterface</plugin>
```

두 설정을 비교하면 교체되는 경계를 볼 수 있다.

```text
실제 로봇: ArduinoHardwareInterface
Gazebo:    gazebo_ros2_control/GazeboSystem
```

Gazebo용 각 조인트에도 controller가 사용할 command/state interface가 필요하다.

```xml
<xacro:ros2_control_joint
  joint_name="joint_1"
  min_pos="${-pi/2}"
  max_pos="${pi/2}"/>
```

이 매크로는 position command interface와 position state interface를 만든다. 실제 로봇 설정과 조인트 이름 및 ROS 단위 범위가 같으므로 같은 `my_controllers.yaml`을 재사용할 수 있다.

실제 로봇 파일에 있던 `servo_min_angle`, `servo_max_angle`, `serial_port`는 Gazebo 설정에 없다. 시뮬레이션에는 ROS 위치를 서보 각도나 CSV로 변환하는 과정이 필요하지 않기 때문이다.

오른쪽 그리퍼는 독립 명령축이 아니라 왼쪽을 따라가는 조인트다.

```xml
<joint name="joint_5_right">
  <param name="mimic">joint_5_left</param>
  <param name="multiplier">1</param>
  <state_interface name="position"/>
</joint>
```

`arm_controller`는 계속 5개 조인트만 명령하고, 오른쪽 그리퍼 상태는 왼쪽 그리퍼의 mimic 관계를 따른다.

### ② libgazebo_ros2_control.so — Gazebo와 ros2_control 연결

파일 아래쪽에는 Gazebo plugin 설정이 있다.

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_manipulator)/config/my_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

이 plugin은 두 시스템 사이의 연결점이다.

```text
ros2_control 쪽
  command interface와 state interface

Gazebo 쪽
  시뮬레이션 조인트와 물리 엔진
```

실제 로봇 launch에서는 `ros2_control_node`를 직접 실행했다. Gazebo 경로에서는 `libgazebo_ros2_control.so`가 로봇을 spawn할 때 controller manager를 만든다. 그래서 `gazebo.launch.py`에는 별도의 `ros2_control_node` 실행 코드가 없다.

plugin은 실제 로봇에서 사용한 것과 같은 `my_controllers.yaml`을 읽는다.

```text
controller_manager update_rate: 100 Hz
joint_state_broadcaster
arm_controller
5개의 position 명령축
```

같은 controller 설정을 재사용한다는 점이 중요하다. 시뮬레이션용 controller를 별도로 만든 것이 아니라 hardware backend만 교체한 것이다.

### ③ gazebo.launch.py — 실행 순서

launch 파일은 먼저 `use_sim_time`을 기본 `true`로 선언한다.

```python
DeclareLaunchArgument(
    "use_sim_time",
    default_value="true",
)
```

Gazebo는 실제 벽시계가 아니라 시뮬레이션 시간을 `/clock`으로 제공한다. 시뮬레이션이 일시 정지되거나 느리게 실행되면 ROS 노드의 시간도 그 흐름을 따라야 하므로 `use_sim_time`을 사용한다.

그다음 실행되는 구성은 다음과 같다.

```text
Gazebo 시작
robot_state_publisher 시작
robot_description 토픽을 사용해 spawn_entity 실행
로봇이 Gazebo에 spawn됨
libgazebo_ros2_control.so가 controller manager 생성
joint_state_broadcaster 시작
arm_controller 시작
```

핵심은 controller를 시작하는 시점이다.

```python
start_controllers = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=spawn_entity,
        on_exit=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ],
    )
)
```

로봇이 spawn되기 전에는 Gazebo plugin과 `/controller_manager`가 아직 준비되지 않았다. 그래서 `spawn_entity`가 끝난 이벤트를 받은 뒤 두 controller를 시작한다.

이 코드를 통해 launch 파일은 단순히 여러 노드를 한꺼번에 실행하는 파일이 아니라, 준비 순서와 의존 관계도 표현한다는 점을 배울 수 있다.

### ④ Gazebo에서 상태가 돌아오는 흐름

실제 로봇의 현재 hardware interface는 위치 센서가 없어서 다음 값을 상태로 보고했다.

```text
hw_positions_ = hw_commands_
```

즉 실제 서보가 막혀도 ROS와 RViz에서는 목표 위치에 도달한 것처럼 보이는 open-loop 구조다.

Gazebo에서는 state가 물리 엔진의 계산 결과에서 나온다.

```text
arm_controller
  -> 이번 제어 주기의 목표 위치 계산
  -> GazeboSystem의 command interface에 기록
  -> Gazebo 물리 엔진이 조인트 상태 계산
  -> GazeboSystem의 state interface 갱신
  -> joint_state_broadcaster
  -> /joint_states
  -> robot_state_publisher
  -> /tf
```

이 흐름에서는 command와 state가 개념적으로 분리된다.

```text
command
  controller가 도달하라고 요청한 위치

state
  물리 계산 후 조인트가 실제로 도달한 위치
```

시뮬레이션 설정과 controller가 이상적이면 둘이 거의 같을 수 있다. 하지만 중력, 충돌, joint limit, 잘못된 관성값 등의 영향이 있으면 차이가 발생할 수 있다.

현재 설정은 effort나 velocity가 아니라 position command interface를 사용한다. 이 방식에서는 GazeboSystem이 목표 위치를 매우 직접적으로 반영할 수 있으므로 실제 서보처럼 가속하고 처지는 현상이 그대로 나타난다고 기대하면 안 된다. 실제 서보의 동역학에 가까운 실험을 하려면 effort 기반 제어, PID, 감쇠, 마찰과 액추에이터 특성을 추가로 모델링해야 한다.

### ⑤ RViz와 Gazebo의 차이 — 운동학에서 동역학으로

RViz에서 조인트 상태를 바꾸면 모델은 즉시 해당 자세로 그려진다. 이것은 주어진 조인트 값으로 링크의 위치를 계산하는 운동학적 표현이다.

Gazebo에서는 다음 URDF 정보가 물리 계산에 사용된다.

```text
visual
  사람에게 보이는 모델

collision
  물체가 서로 닿았는지 계산하는 형상

inertial
  질량, 무게중심, 관성 텐서

joint limit
  관절의 위치, 속도, 힘 제한
```

예를 들어 visual mesh가 정확해도 collision이나 inertial 값이 잘못되면 Gazebo에서 다음 문제가 나타날 수 있다.

- 링크가 떨리거나 튄다.
- 관절이 목표 위치를 유지하지 못한다.
- 충돌 판정이 실제 형상과 다르게 보인다.
- 모델이 지나치게 가볍거나 무겁게 움직인다.
- 시뮬레이션 계산이 불안정해진다.

현재 공통 모델의 질량과 관성은 작은 근삿값이다. 따라서 지금 단계의 Gazebo 목표는 실제 로봇의 동작을 정밀하게 복제하는 것이 아니다. 먼저 model-controller-hardware 연결이 올바른지 확인하고, 이후 CAD나 측정값을 사용해 물리 파라미터를 보정해야 한다.

### ⑥ 첫 실행과 확인 순서

터미널 1에서 Gazebo를 실행한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_manipulator gazebo.launch.py
```

Gazebo와 함께 ROS TF 모델을 RViz에서도 보고 싶으면 다른 터미널에서 RViz만 따로 실행한다.

```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/manipulator/my_manipulator/rviz/display.rviz
```

로봇 spawn이 끝난 후 터미널 2에서 controller를 확인한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
```

예상 상태:

```text
joint_state_broadcaster       active
arm_controller                active
```

Gazebo가 계산한 조인트 상태를 확인한다.

```bash
ros2 topic echo /joint_states
```

다음 trajectory 명령을 전송한다.

```bash
ros2 topic pub --once \
  /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5_left"],
    points: [{
      positions: [0.785, 1.571, -1.571, 0.0, 0.004],
      time_from_start: {sec: 2, nanosec: 0}
    }]
  }'
```

회전 조인트 단위는 radian이고 그리퍼 단위는 meter다. 실제 로봇에 보낸 것과 같은 ROS 명령이지만, 이번에는 Arduino용 각도로 변환되지 않고 GazeboSystem으로 전달된다.

### Gazebo가 대신 검증할 수 없는 것

Gazebo가 움직인다고 실제 로봇도 반드시 같은 방식으로 움직이는 것은 아니다. 현재 시뮬레이션은 다음 현실 요소를 정확하게 재현하지 않는다.

- 서보의 실제 토크와 속도
- 기어 백래시와 마찰
- 링크 조립 오차
- 전원 부족과 전압 강하
- USB 시리얼 지연과 데이터 손실
- Arduino 펌웨어의 버퍼와 명령 처리
- 현재 position interface로 생략된 서보의 PID와 동특성

따라서 Gazebo는 실제 로봇을 없애는 도구가 아니라, 모델과 controller를 실제 하드웨어보다 안전하고 반복 가능한 조건에서 검증하는 도구다.

---

## 7. MoveIt 2 적용 — 목표 자세에서 조인트 궤적 만들기

### 왜 지금 MoveIt 2를 배우는가

6장까지는 사람이 모든 조인트의 목표값을 직접 정해 `JointTrajectory` 메시지를 보냈다.

```text
사람이 joint_1 ~ joint_5_left의 목표값 결정
  -> arm_controller
  -> Gazebo 또는 실제 로봇
```

이 방식은 제어 경로를 검증하기에는 좋지만, 실제 작업을 지시하기에는 불편하다. 예를 들어 "그리퍼를 물체 앞의 특정 위치와 방향으로 이동하라"는 명령을 수행하려면 다음 계산이 더 필요하다.

```text
목표 위치와 방향을 만족하는 조인트 값은 무엇인가?          역기구학(IK)
현재 자세에서 목표 자세까지 어떤 경로로 움직여야 하는가?     경로 계획
그 경로에서 로봇 자신이나 주변 물체와 부딪히지 않는가?       충돌 검사
계산한 경로를 controller가 실행할 수 있는 궤적으로 만들 수 있는가?
```

MoveIt 2가 이 계산을 담당한다. 지금까지 만든 URDF, TF, `/joint_states`, `arm_controller`를 버리고 새로운 제어 시스템으로 바꾸는 것이 아니다. 그 위에 경로를 계획하는 상위 계층을 추가한다.

```text
목표 자세(Pose) 또는 목표 조인트 값
  -> MoveIt 2
     - 현재 상태 확인
     - IK 계산
     - 충돌 검사
     - 경로 계획
  -> JointTrajectory
  -> 기존 arm_controller
  -> 기존 ros2_control command interface
  -> GazeboSystem 또는 ArduinoHardwareInterface
```

따라서 MoveIt 아래쪽의 실행 대상만 바꾸면 같은 계획 구조를 시뮬레이션과 실제 로봇에서 재사용할 수 있다.

### 기존 패키지와 새 MoveIt 패키지의 관계

MoveIt 설정은 기존 `my_manipulator` 패키지에 모두 넣지 않고 `my_manipulator_moveit`라는 별도 ROS 2 패키지로 만든다. 두 패키지는 같은 GitHub 리포지토리 `manipulator` 안에서 함께 버전 관리한다.

두 패키지는 다음처럼 역할을 나눈다.

```text
my_manipulator
  로봇 자체를 설명하고 움직이는 패키지
  - URDF/Xacro와 mesh
  - ros2_control 설정
  - Gazebo 실행
  - 실제 Arduino hardware interface

my_manipulator_moveit
  그 로봇의 경로 계획 방법을 설명하는 패키지
  - planning group
  - IK solver
  - 충돌 검사 설정
  - 경로 planner
  - 계획 결과를 전달할 controller 정보
  - move_group과 MoveIt용 RViz 실행
```

워크스페이스에 두 패키지가 함께 놓이는 전체 구조는 다음과 같다. Setup Assistant 버전에 따라 생성되는 launch 파일의 세부 이름은 달라질 수 있으므로 아래는 생성 전 예상 구조다. 실제 생성 후에는 만들어진 파일을 기준으로 이 목록을 갱신한다.

```text
ros2_ws/src/
└── manipulator/                  # GitHub repository
    ├── README.md
    ├── LEARNING_NOTES.md
    ├── my_manipulator/           # base ROS 2 package
    │   ├── urdf/
    │   │   ├── manipulator.xacro
    │   │   ├── manipulator_sim.urdf.xacro
    │   │   └── manipulator_real.urdf.xacro
    │   ├── meshes/
    │   ├── config/
    │   │   └── my_controllers.yaml
    │   ├── launch/
    │   │   ├── display.launch.py
    │   │   ├── gazebo.launch.py
    │   │   └── real_robot.launch.py
    │   └── src/
    │       └── Arduino hardware interface와 serial driver
    └── my_manipulator_moveit/    # MoveIt 2 config package
        ├── package.xml
        ├── CMakeLists.txt
        ├── .setup_assistant
        ├── config/
        │   ├── manipulator.srdf
        │   ├── kinematics.yaml
        │   ├── joint_limits.yaml
        │   ├── ompl_planning.yaml
        │   ├── moveit_controllers.yaml
        │   └── moveit.rviz
        └── launch/
            ├── demo.launch.py
            ├── move_group.launch.py
            ├── moveit_rviz.launch.py
            └── gazebo_moveit.launch.py
```

`my_manipulator_moveit`가 별도 패키지여도 로봇 모델을 새로 만드는 것은 아니다. 형상과 조인트의 원본은 계속 `my_manipulator` 패키지에 두고, MoveIt 패키지가 그 모델을 불러와 계획에 필요한 의미와 설정을 덧붙인다.

### 전체 실행 구조와 각 구성의 역할

MoveIt 실습에서 RViz와 Gazebo에 같은 로봇이 보이지만 두 프로그램이 같은 일을 하는 것은 아니다.

| 구성 | 역할 | 하지 않는 일 |
|---|---|---|
| RViz MotionPlanning 패널 | 목표 자세 입력, 계획 경로 미리보기, MoveIt Planning Scene 확인 | 중력·접촉 같은 물리 계산 |
| MoveIt `move_group` | 현재 상태를 읽고 IK·충돌 검사·경로 계획 수행 | 모터나 Gazebo 조인트를 직접 구동 |
| Gazebo | 전달받은 궤적을 물리 환경에서 실행 | 목표 자세까지의 충돌 회피 경로 계획 |
| ros2_control | 궤적을 각 제어 주기의 조인트 명령으로 실행 | 작업 공간의 목표 자세 결정 |

RViz는 단순 모델 뷰어가 아니라 ROS와 MoveIt 내부 상태를 들여다보는 계기판이다. RViz의 인터랙티브 마커로 목표 자세를 지정해도 Gazebo 로봇이 즉시 움직이지 않는다. 먼저 MoveIt이 경로를 계획하고, RViz에서 계획 결과를 확인한 다음 `Execute`해야 궤적이 ros2_control controller로 전달된다.

```text
RViz에서 목표 자세 입력
  -> move_group이 현재 /joint_states 확인
  -> IK와 충돌 없는 경로 계산
  -> RViz에서 계획 경로 미리보기
  -> Execute
  -> arm_controller
  -> Gazebo 또는 실제 로봇
```

RViz는 목표를 넣는 여러 방법 중 하나일 뿐이다. 연결을 검증한 뒤에는 Python/C++ 노드, 카메라 인식 결과 또는 작업 명령이 MoveIt에 목표를 전달할 수 있으므로 RViz 없이도 실행할 수 있다.

### MoveIt이 사용하는 URDF와 SRDF

MoveIt은 기존 URDF와 함께 SRDF(Semantic Robot Description Format)를 사용한다.

```text
URDF
  link와 joint의 물리적 구조
  joint 축과 limit
  visual과 collision 형상

SRDF
  어떤 joint들을 하나의 planning group으로 볼지
  end effector가 어느 group과 link에 연결되는지
  기본 자세 이름
  항상 맞닿아 있어 충돌 검사에서 제외할 link 쌍
```

즉 URDF가 로봇의 몸을 설명한다면 SRDF는 MoveIt이 그 몸을 어떤 단위로 계획할지 설명한다.

이 매니퓰레이터의 첫 설정은 다음처럼 나눈다.

```text
arm planning group
  joint_1
  joint_2
  joint_3
  joint_4

gripper planning group
  joint_5_left
  joint_5_right는 mimic joint이므로 독립 명령축으로 취급하지 않음
```

첫 실습에서는 팔의 목표 link를 `link_5`로 사용한다. `link_5`는 손목 끝이자 그리퍼 부모 link다. 나중에 집기 위치를 더 명확하게 표현하려면 두 손가락 사이의 중심에 `tool0` 또는 `tcp_link`라는 고정 link를 추가하고 그 link를 목표로 삼는 편이 좋다.

### 기존 my_controllers.yaml은 무엇을 만들고 있는가

MoveIt과 controller를 연결하기 전에 현재 사용 중인 `my_manipulator/config/my_controllers.yaml`의 역할을 다시 확인한다.

```yaml
controller_manager:
  ros__parameters:
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
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

여기서 이름과 종류를 구분해야 한다.

```text
arm_controller
  이 프로젝트에서 controller에 붙인 이름

joint_trajectory_controller/JointTrajectoryController
  ros2_control이 불러오는 controller plugin의 종류
```

이 YAML은 controller manager가 읽는다. 그 결과 `arm_controller`라는 실제 controller가 생성되고, 이 controller가 다섯 조인트의 position command interface를 사용한다.

```text
my_controllers.yaml
  -> controller manager가 읽음
  -> joint_trajectory_controller plugin으로 arm_controller 생성
  -> joint_1 ~ joint_5_left의 command interface 사용
```

### 지금까지 사용한 토픽과 controller가 제공하는 action

6장에서는 다음 토픽으로 `JointTrajectory` 메시지를 직접 보냈다.

```text
/arm_controller/joint_trajectory
```

전체 이름은 아래 두 부분으로 만들어진다.

```text
controller 이름
  arm_controller

controller가 제공하는 토픽 이름
  joint_trajectory

결과
  /arm_controller/joint_trajectory
```

이 토픽 방식은 명령을 한 번 보내는 데는 간단하지만, 명령을 보낸 쪽이 실행 과정과 성공·실패 결과를 받기 어렵다. 실행 결과를 돌려주는 별도의 응답 통로가 없기 때문이다.

`JointTrajectoryController`는 토픽 외에도 다음 action server를 기본으로 제공한다.

```text
/arm_controller/follow_joint_trajectory
```

이 이름도 같은 방법으로 만들어진다.

```text
controller 이름
  arm_controller

action namespace
  follow_joint_trajectory

결과
  /arm_controller/follow_joint_trajectory
```

`FollowJointTrajectory`는 새로운 controller의 이름이 아니다. `control_msgs/action/FollowJointTrajectory`라는 action 통신 형식이다. 내부 목표에는 지금까지 사용한 것과 같은 `trajectory_msgs/msg/JointTrajectory`가 들어가지만, action에는 다음 기능이 추가된다.

```text
goal
  실행할 trajectory 전달

feedback
  trajectory 실행 중 상태 전달

result
  목표 도달, 실패 또는 취소 결과 전달

cancel
  실행 중인 목표 취소
```

두 명령 방식을 비교하면 다음과 같다.

```text
지금까지의 직접 명령
  ros2 topic pub
    -> /arm_controller/joint_trajectory
    -> trajectory 실행
    -> 명령을 보낸 쪽에서 최종 결과를 직접 받지 않음

MoveIt의 실행
  move_group의 action client
    -> /arm_controller/follow_joint_trajectory
    -> 같은 controller가 trajectory 실행
    -> feedback과 최종 결과를 move_group이 받음
```

MoveIt은 계획한 경로가 실제로 끝났는지 감시해야 하므로 action 방식을 사용한다. 새로운 ros2_control controller를 만드는 것이 아니라, 이미 사용 중인 `arm_controller`가 제공하는 다른 명령 통로를 사용하는 것이다.

실행 중인 action은 다음 명령으로 확인할 수 있다.

```bash
ros2 action list
ros2 action info /arm_controller/follow_joint_trajectory
```

### my_controllers.yaml과 moveit_controllers.yaml의 차이

이제 새 MoveIt 패키지의 `config/moveit_controllers.yaml`이 필요한 이유를 볼 수 있다.

| 파일 | 읽는 주체 | 역할 |
|---|---|---|
| `my_manipulator/config/my_controllers.yaml` | ros2_control의 controller manager | controller plugin을 실제로 생성하고 사용할 joint와 command/state interface를 정함 |
| `my_manipulator_moveit/config/moveit_controllers.yaml` | MoveIt의 `move_group` | 이미 실행 중인 controller의 action 주소와 담당 joint를 알려줌 |

두 파일은 같은 controller를 서로 다른 쪽에서 설명하므로 내용이 다르다. `moveit_controllers.yaml`이 controller를 새로 생성하거나 hardware interface에 연결하는 것은 아니다.

이 프로젝트에서 사용할 MoveIt 쪽 설정은 다음 형태가 된다.

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller

  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5_left
```

각 항목은 다음을 의미한다.

```text
controller_names
  MoveIt이 사용할 수 있는 controller 이름 목록

arm_controller
  my_controllers.yaml에서 실제로 생성한 controller와 맞출 이름

action_ns: follow_joint_trajectory
  controller가 제공하는 action namespace

type: FollowJointTrajectory
  MoveIt이 사용할 action 통신 형식

joints
  이 controller가 명령할 수 있는 joint 목록
```

따라서 MoveIt이 최종적으로 찾아가는 action 주소는 두 값을 합친 결과다.

```text
controller 이름 + action namespace

arm_controller + follow_joint_trajectory
  -> /arm_controller/follow_joint_trajectory
```

여기서 “기존 controller 이름을 재사용한다”는 말은 `my_controllers.yaml` 전체를 복사한다는 뜻이 아니다. 두 파일에 적힌 `arm_controller`라는 이름을 맞춰, MoveIt이 이미 실행 중인 controller의 action server를 찾아가게 한다는 뜻이다.

### arm 그룹과 기존 controller의 joint 수 문제

MoveIt의 첫 planning group은 다음 네 조인트로 구성한다.

```text
arm
  joint_1
  joint_2
  joint_3
  joint_4
```

하지만 현재 `arm_controller`는 그리퍼까지 포함한 다섯 조인트를 제어한다.

```text
arm_controller
  joint_1
  joint_2
  joint_3
  joint_4
  joint_5_left
```

기본 설정의 `JointTrajectoryController`는 controller에 등록된 모든 joint가 trajectory에 들어오기를 요구한다. 따라서 MoveIt이 `arm` 그룹의 네 조인트만 계획해 보내면 그대로는 거부될 수 있다.

첫 실습에서는 controller를 팔과 그리퍼로 나누지 않고, 기존 controller 하나를 유지하면서 다음 옵션을 `my_controllers.yaml`에 추가한다.

```yaml
arm_controller:
  ros__parameters:
    allow_partial_joints_goal: true
```

이 설정은 일부 joint만 담긴 action goal도 허용한다.

```text
arm 계획 실행
  joint_1 ~ joint_4만 전달
  joint_5_left는 현재 위치 유지

gripper 계획 실행
  joint_5_left만 전달
  joint_1 ~ joint_4는 현재 위치 유지
```

이 방식은 현재 controller 구조를 최소한으로 변경해 MoveIt 연결을 배우기 위한 선택이다. 이후 팔과 그리퍼를 독립적으로 운용할 필요가 커지면 `arm_trajectory_controller`와 gripper용 controller를 분리하는 구조를 다시 검토할 수 있다.

### my_manipulator_moveit의 주요 설정 파일

MoveIt Setup Assistant로 생성할 `my_manipulator_moveit`의 주요 파일은 다음 역할을 맡는다.

```text
package.xml
  MoveIt 실행에 필요한 ROS 2 패키지 의존성 선언

CMakeLists.txt
  config와 launch 디렉터리를 install 공간에 복사

.setup_assistant
  Setup Assistant가 설정 패키지를 다시 열 때 사용하는 작업 정보

config/manipulator.srdf
  arm/gripper group, 기본 자세, end effector, 충돌 제외 관계

config/kinematics.yaml
  arm group이 사용할 IK solver와 탐색 설정

config/joint_limits.yaml
  MoveIt에서 사용할 속도·가속도 제한

config/ompl_planning.yaml
  OMPL 경로 planner 설정

config/moveit_controllers.yaml
  move_group이 기존 controller action을 찾아가기 위한 정보

config/moveit.rviz
  MotionPlanning 패널과 MoveIt 시각화 설정

launch/move_group.launch.py
  경로 계획의 중심 노드인 move_group 실행

launch/moveit_rviz.launch.py
  MotionPlanning 패널이 포함된 RViz 실행

launch/demo.launch.py
  MoveIt 설정을 단독으로 확인하기 위한 구성 요소들을 묶어 실행

launch/gazebo_moveit.launch.py
  기존 Gazebo 실행과 move_group, MoveIt RViz를 함께 연결하기 위해 추가할 launch
```

### 적용 순서

아래 순서는 이 리포지토리 상태에서 그대로 따라가기 위한 실습 경로다. 핵심은 **MoveIt 설정은 생성하고**, **Gazebo/실제 로봇을 실행하는 controller는 기존 `my_manipulator` 쪽 것을 계속 사용한다**는 점이다.

```text
1. 의존성 설치와 현재 로봇 모델 점검
2. MoveIt Setup Assistant로 SRDF와 planning 설정 생성
3. 생성된 설정에서 controller 이름을 arm_controller로 맞춤
4. MoveIt 단독 Plan 확인
5. Gazebo의 arm_controller action으로 Execute 연결
6. 장애물과 코드 제어로 확장
7. 실제 로봇 launch로 하드웨어만 교체
```

#### 1단계 — 설치와 모델 사전 점검

MoveIt 2와 Setup Assistant를 설치한다.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant
```

현재 워크스페이스를 빌드하고 환경을 읽는다.

```bash
cd ~/ros2_ws
colcon build --packages-select my_manipulator my_manipulator_moveit
source install/setup.bash
```

xacro가 오류 없이 URDF로 변환되는지 확인한다.

```bash
xacro src/manipulator/my_manipulator/urdf/manipulator.xacro > /tmp/my_manipulator.urdf
check_urdf /tmp/my_manipulator.urdf
```

#### 2단계 — MoveIt 설정 패키지 생성

Setup Assistant를 실행한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

GUI에서는 다음 순서로 진행한다.

```text
Start
  Create New MoveIt Configuration Package 선택
  Load Files에서 아래 xacro 선택
  ~/ros2_ws/src/manipulator/my_manipulator/urdf/manipulator.xacro

Self-Collisions
  Generate Collision Matrix 실행

Planning Groups
  arm 그룹 생성
    kinematic chain 또는 joint 목록으로 joint_1 ~ joint_4 포함
    목표 link는 우선 link_5 기준으로 생각한다.

  gripper 그룹 생성
    joint 목록으로 joint_5_left 포함
    joint_5_right는 mimic joint라 독립 명령축으로 넣지 않는다.

Robot Poses
  home 같은 기본 자세를 하나 등록

End Effectors
  gripper 그룹을 end effector로 등록
  parent link는 link_5 사용

ROS 2 Controllers
  controller 이름은 arm_controller 사용
  type은 FollowJointTrajectory 사용
  joints는 joint_1, joint_2, joint_3, joint_4, joint_5_left 사용

Configuration Files
  package path를 아래 경로로 지정
  ~/ros2_ws/src/manipulator/my_manipulator_moveit
  Generate Package 실행
```

Setup Assistant가 기존 `package.xml`, `CMakeLists.txt`, `config/moveit_controllers.yaml`을 덮어쓸 수 있다. 생성 후에는 다음 항목을 다시 확인한다.

```text
패키지 이름: my_manipulator_moveit
기본 로봇 패키지 참조: my_manipulator
controller 이름: arm_controller
action namespace: follow_joint_trajectory
```

이 단계에서는 아직 Gazebo 물리를 검증하려는 것이 아니라, MoveIt이 로봇의 관절 구조와 충돌 형상을 올바르게 읽는지 확인한다.

생성 후 다시 빌드한다.

```bash
cd ~/ros2_ws
colcon build --packages-select my_manipulator my_manipulator_moveit
source install/setup.bash
```

#### 2.5단계 — 생성된 controller 설정 확인

`my_manipulator_moveit/config/moveit_controllers.yaml`은 최소한 다음 구조여야 한다.

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5_left
```

`my_manipulator/config/my_controllers.yaml`에는 MoveIt이 arm 그룹의 일부 joint만 보내도 거부하지 않도록 다음 옵션이 들어 있어야 한다.

```yaml
arm_controller:
  ros__parameters:
    allow_partial_joints_goal: true
```

여기서 다시 한 번 이름을 구분한다.

```text
arm_controller
  우리가 만든 controller 인스턴스 이름

joint_trajectory_controller/JointTrajectoryController
  arm_controller를 만들 때 사용하는 ros2_control plugin 타입

FollowJointTrajectory
  MoveIt이 arm_controller로 목표를 보낼 때 사용하는 action 형식
```

#### 3단계 — MoveIt 단독 계획 확인

Setup Assistant가 생성한 launch 파일 이름은 버전에 따라 조금 다를 수 있다. 보통은 먼저 demo launch로 MoveIt 설정이 단독으로 열리는지 확인한다.

```bash
ros2 launch my_manipulator_moveit demo.launch.py
```

RViz MotionPlanning 패널에서 다음을 확인한다.

```text
Planning Group에서 arm을 선택할 수 있는가?
Planning Group에서 gripper를 선택할 수 있는가?
joint-space 목표 또는 named state 목표로 Plan이 되는가?
현재 자세와 목표 자세가 joint limit 안에 있는가?
```

`Plan`은 계산과 미리보기만 하며 controller로 명령을 보내지 않는다. 로봇 일부가 반짝이거나 흐릿한 궤적이 보이는 것은 보통 RViz가 계획된 경로 또는 planning scene 상태를 표시하는 것이지, 실제 실행은 아니다. 실제 실행은 `Execute`를 눌렀을 때 `arm_controller` action으로 전달된다.

`link_5`의 목표 자세 마커는 IK 설정이 있어야 의미 있게 동작한다. 현재 arm은 4축이라 6D pose 전체를 자유롭게 맞출 수 없으므로 `kinematics.yaml`에서는 우선 위치만 맞추는 `position_only_ik`를 사용한다.

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  position_only_ik: true
```

따라서 첫 검증 순서는 다음처럼 잡는다.

```text
1. arm 그룹 선택
2. joint-space 목표 또는 home 같은 named state로 Plan
3. 계획 궤적 미리보기 확인
4. link_5 pose marker는 IK 경고가 사라진 뒤 작은 이동부터 확인
```

#### 4단계 — Gazebo 실행과 연결

Gazebo가 로봇, `/joint_states`, controller manager를 제공하고 MoveIt은 계획 노드와 RViz만 제공하도록 구성한다.

```text
gazebo.launch.py
  -> Gazebo 로봇
  -> joint_state_broadcaster
  -> arm_controller
  -> /joint_states

MoveIt launch
  -> move_group
  -> RViz MotionPlanning
  -> 기존 arm_controller의 action 사용
```

MoveIt의 `demo.launch.py`는 fake hardware와 별도의 `ros2_control_node`를 함께 시작한다. Gazebo와 같이 켜면 MoveIt이 Gazebo의 controller가 아니라 fake controller로 trajectory를 보낼 수 있다. 따라서 Gazebo 연동에서는 `demo.launch.py`를 쓰지 않고, `move_group`과 MoveIt RViz만 실행하며 로봇 상태와 controller는 기존 Gazebo 경로의 것을 사용한다.

Gazebo와 MoveIt을 함께 확인할 때는 먼저 이 명령 하나만 사용한다.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_manipulator_moveit gazebo_moveit.launch.py
```

이 명령은 내부에서 다음 세 launch를 순서대로 묶어 실행한다.

```text
my_manipulator/gazebo.launch.py
  Gazebo 로봇과 GazeboSystem controller manager 실행

my_manipulator_moveit/move_group.launch.py
  MoveIt의 계획 노드 실행

my_manipulator_moveit/moveit_rviz.launch.py
  MotionPlanning 패널이 있는 RViz 실행
```

이 통합 launch에서는 `move_group`과 RViz가 Gazebo의 `/clock`을 따르도록 `use_sim_time:=true`로 실행되고, MoveIt은 controller를 직접 시작하거나 전환하지 않는다. 이미 Gazebo 쪽에서 active 상태가 된 `arm_controller`에 trajectory action만 보낸다.

실행 후 RViz에서 다음 순서로 확인한다.

```text
1. MotionPlanning 패널에서 Planning Group을 arm으로 선택한다.
2. Start State는 current로 둔다.
3. Goal State는 작은 joint-space 목표나 home 같은 named state로 둔다.
4. Plan을 눌러 궤적 미리보기가 나오는지 확인한다.
5. Execute를 눌러 Gazebo 로봇이 움직이는지 확인한다.
```

Gazebo 로봇이 너무 천천히 움직이면 `Plan`을 누르기 전에 MotionPlanning 패널의 `Velocity Scaling` 값을 올린다. 예를 들어 `0.1`이면 최대 조인트 속도의 10%만 사용하므로 계획된 trajectory 시간이 길어진다. `0.5`로 바꾸고 다시 `Plan`하면 waypoint들의 `time_from_start`가 더 짧게 잡히고, `Execute` 때 Gazebo 로봇도 더 빠르게 움직인다.

여기서 중요한 구분은 다음과 같다.

```text
Plan
  MoveIt 내부에서 경로만 계산하고 RViz에 미리보기로 표시한다.
  Gazebo 로봇은 아직 움직이지 않는다.

Execute
  계획된 JointTrajectory를 /arm_controller/follow_joint_trajectory action으로 보낸다.
  이때 Gazebo의 arm_controller가 받아 로봇을 움직인다.
```

정상 동작의 기준은 다음과 같다.

```text
Gazebo
  로봇 모델이 보이고 Execute 후 조인트가 움직인다.

RViz
  planned path가 보이고 현재 로봇 상태가 /joint_states를 따라 갱신된다.

터미널 로그
  arm_controller가 active 상태로 올라오고
  MoveItSimpleControllerManager가 arm_controller를 추가했다는 로그가 나온다.
```

움직이지 않을 때도 이 단계 안에서 여러 명령으로 흐름을 쪼개기보다는, 먼저 `demo.launch.py`를 같이 켜지 않았는지와 `Execute`까지 눌렀는지만 확인한다. 추가 진단 명령은 실제로 문제가 생겼을 때 별도로 확인한다.

첫 통합 목표는 다음과 같다.

```text
1. RViz에서 link_5의 도달 가능한 목표 자세를 지정한다.
2. Plan으로 예상 경로를 확인한다.
3. Execute를 누른다.
4. MoveIt이 FollowJointTrajectory action으로 궤적을 보낸다.
5. Gazebo의 로봇이 계획된 경로를 따라 움직인다.
6. /joint_states가 다시 MoveIt의 현재 상태로 반영된다.
```

#### 5단계 — Planning Scene과 장애물

Gazebo 화면에 책상이나 상자가 보인다고 MoveIt이 자동으로 그것을 장애물로 아는 것은 아니다. Gazebo의 물리 world와 MoveIt의 Planning Scene은 별개의 세계 표현이다.

```text
Gazebo world
  물리 엔진이 충돌과 접촉을 계산하는 환경

MoveIt Planning Scene
  경로 계획 중 충돌을 검사하는 환경
```

따라서 장애물 회피 실습에서는 같은 물체를 MoveIt Planning Scene에 collision object로 추가해야 한다. RViz에서는 Gazebo 화면이 아니라 MoveIt이 실제로 알고 있는 충돌 환경을 확인한다.

첫 실습은 RViz에서 Planning Scene에 박스 하나를 직접 추가하는 방식으로 진행한다.

```text
1. 4단계와 같은 gazebo_moveit.launch.py를 실행한다.
2. RViz MotionPlanning 패널에서 Scene Objects 탭을 연다.
3. Box 형태의 collision object를 하나 추가한다.
4. 로봇 팔이 지나갈 법한 위치에 박스를 둔다.
5. Publish Scene 또는 Apply 버튼으로 Planning Scene에 반영한다.
6. MotionPlanning 탭으로 돌아가 같은 목표에 대해 다시 Plan한다.
```

이때 RViz에 보이는 박스는 MoveIt의 충돌 검사에 쓰이는 물체다. Gazebo 물리 world에 자동으로 생긴 물체가 아니므로 Gazebo 화면에는 보이지 않을 수 있다. 반대로 Gazebo 화면에 물체가 있어도 MoveIt Planning Scene에 추가하지 않으면 MoveIt은 그 물체를 피하지 않는다.

성공 기준은 다음과 같다.

```text
장애물 없는 상태
  목표까지 직선에 가까운 경로가 계획될 수 있다.

장애물 추가 후
  같은 목표를 다시 Plan했을 때 경로가 장애물을 피해 돌아가거나,
  피할 수 없는 경우 planning 실패가 난다.
```

#### 6단계 — Named State 관리와 코드 목표 전달

RViz 검증이 끝나면 매번 목표를 손으로 끌어 움직이기보다, 자주 쓰는 자세를 이름으로 저장해 두는 것이 좋다. 이처럼 이름이 붙은 조인트 자세를 MoveIt에서는 named state 또는 group state로 다룬다.

현재 설정에는 `home`이라는 arm group 상태가 하나 들어 있다. 이 정보는 `my_manipulator_moveit/config/manipulator.srdf`에 저장된다.

```xml
<group_state name="home" group="arm">
    <joint name="joint_1" value="0"/>
    <joint name="joint_2" value="0"/>
    <joint name="joint_3" value="0"/>
    <joint name="joint_4" value="0"/>
</group_state>
```

새 named state를 만드는 방법은 두 가지다.

Setup Assistant를 다시 열 때는 다음 명령을 사용한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

```text
MoveIt Setup Assistant
  Robot Poses 단계에서 arm group을 선택한다.
  joint 값을 원하는 자세로 맞춘다.
  ready, pick_prepose 같은 이름으로 저장한다.
  Generate Package를 다시 실행하면 SRDF에 group_state로 기록된다.

SRDF 직접 수정
  manipulator.srdf에 <group_state> 블록을 추가한다.
  group 이름과 joint 이름은 기존 planning group과 정확히 맞춘다.
  파일 수정 후 colcon build와 source를 다시 수행한다.
```

예를 들어 `ready` 자세를 추가한다면 구조는 다음과 같다.

```xml
<group_state name="ready" group="arm">
    <joint name="joint_1" value="0.0"/>
    <joint name="joint_2" value="0.4"/>
    <joint name="joint_3" value="-0.8"/>
    <joint name="joint_4" value="0.2"/>
</group_state>
```

이 방식의 장점은 목표를 줄 때마다 엔드이펙터 pose나 각 조인트 값을 다시 계산하지 않아도 된다는 점이다. 코드에서는 이름만 선택하면 된다.

```text
코드
  -> arm group 선택
  -> named target "home" 선택
  -> Plan
  -> Execute
  -> 기존 /arm_controller/follow_joint_trajectory action으로 전달
```

RViz MotionPlanning 패널에서 고른 named state와 코드에서 쓰는 named target은 같은 SRDF 정보를 기준으로 한다. 따라서 자주 쓰는 자세는 `home`, `ready`, `pick_prepose`처럼 SRDF의 `group_state`로 저장해 두는 것이 좋다.

주의할 점은 RViz에서 임시로 움직인 목표 자세가 자동으로 SRDF named state가 되는 것은 아니라는 점이다. RViz의 Stored States나 warehouse 기능은 RViz/DB 쪽 저장 기능에 가깝고, 이 프로젝트 설정 파일에 항상 남는 named state와는 구분해서 보는 것이 좋다.

터미널에서 바로 `home`이라는 이름만 넣어 MoveIt 계획과 실행을 시키는 기본 명령은 없다. 터미널에서 `ros2 action send_goal /arm_controller/follow_joint_trajectory ...`를 직접 쓰면 controller로 조인트 trajectory를 보낼 수는 있지만, 이 경우 MoveIt의 충돌 검사와 planning을 거치지 않는다. MoveIt을 거치려면 작은 Python/C++ 노드를 만들어 `setNamedTarget("home")`처럼 named state를 목표로 넣고 plan/execute를 호출하는 흐름을 사용한다.

이때부터 RViz는 필수 실행 요소가 아니라 필요할 때 켜는 디버깅 도구가 된다.

#### 7단계 — 실제 로봇으로 교체

Gazebo 연결이 검증되면 아래쪽 실행 대상을 `real_robot.launch.py`로 바꾼다.

```text
MoveIt 계획 계층은 유지
GazeboSystem 대신 ArduinoHardwareInterface 사용
같은 arm_controller action 사용
```

실제 로봇에서는 현재 엔코더 피드백이 없어 명령 위치를 현재 위치로 간주한다. 따라서 MoveIt 화면에서 정상으로 보여도 로봇이 물리적으로 막혔는지는 알 수 없다. 처음에는 낮은 속도와 좁은 작업 범위에서 시험하고, 비상 정지와 전원 차단 수단을 준비해야 한다.

MoveIt RViz에서 실제 로봇을 움직일 때는 `my_manipulator_moveit`의 실제 로봇용 통합 launch를 사용한다. 구조는 Gazebo 통합 launch와 같지만, 포함하는 실행 대상만 바뀐다.

```text
gazebo_moveit.launch.py
  -> my_manipulator/gazebo.launch.py
  -> GazeboSystem

real_moveit.launch.py
  -> my_manipulator/real_robot.launch.py
  -> ArduinoHardwareInterface
```

먼저 실제 시리얼 포트를 열지 않는 dry-run으로 MoveIt과 controller 연결을 확인한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_manipulator_moveit real_moveit.launch.py dry_run:=true
```

이 상태에서 RViz MotionPlanning 패널로 `Plan`과 `Execute`를 누르면, 하드웨어 인터페이스가 Arduino로 실제 전송하지 않고 CSV 명령을 로그로 출력한다. 계획과 실행 흐름이 맞는지 확인한 뒤 실제 로봇을 연결한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_manipulator_moveit real_moveit.launch.py serial_port:=/dev/ttyACM0
```

포트 이름은 환경에 따라 `/dev/ttyUSB0`, `/dev/ttyACM0`처럼 달라질 수 있다. 처음 실제 실행할 때는 RViz MotionPlanning 패널에서 `Velocity Scaling`을 낮게 두고, `home`처럼 이미 검증한 named state나 아주 가까운 목표만 사용한다.

### 단계별 성공 기준

| 단계 | 성공 기준 |
|---|---|
| 설정 생성 | Setup Assistant에서 모델과 planning group을 오류 없이 읽음 |
| 계획 | RViz에서 `arm` 목표에 대해 Plan이 성공하고 궤적이 보임 |
| 시뮬레이션 실행 | Execute 후 Gazebo 로봇이 움직이고 controller action이 성공함 |
| 장애물 회피 | Planning Scene 장애물을 포함한 경로가 충돌 없이 생성됨 |
| 코드 제어 | RViz 목표 마커 없이 노드가 pose goal을 보내 실행함 |
| 실제 로봇 | 제한된 속도에서 계획 궤적을 실제 서보가 안전하게 실행함 |

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
- Arduino 펌웨어의 수신 버퍼 오버플로 정책 보강과 watchdog 구현
- 통신 단절 시 서보 정지/토크 해제 정책
- 엔코더를 사용할 경우 closed-loop state feedback 추가
- 장기적으로 Gazebo Classic에서 modern Gazebo로 전환

이 항목들은 소프트웨어 누락이라기보다 실제 기구와 전장 사양을 알아야 결정할 수 있는 부분이다.

---

## 부록 C. 실행·검증 명령 모음

### RViz

```bash
ros2 launch my_manipulator display.launch.py
```

- Fixed Frame이 `world`인지
- 5개 슬라이더가 나타나는지
- 왼쪽 그리퍼 조작 시 오른쪽도 대칭 이동하는지

### Gazebo

```bash
ros2 launch my_manipulator gazebo.launch.py
rviz2 -d ~/ros2_ws/src/manipulator/my_manipulator/rviz/display.rviz
ros2 control list_controllers
ros2 topic echo /joint_states
```

- 두 컨트롤러가 active인지
- RViz를 별도 실행했을 때 `/joint_states` → `/tf` 흐름으로 모델이 움직이는지
- trajectory 명령 후 5개 명령축이 목표값으로 이동하는지
- `joint_5_right`가 `joint_5_left`를 따라가는지

### 실제 로봇

```bash
ros2 launch my_manipulator real_robot.launch.py serial_port:=/dev/ttyACM0
rviz2 -d ~/ros2_ws/src/manipulator/my_manipulator/rviz/display.rviz
```

- Arduino 없이 테스트할 때는 `dry_run:=true`로 실행했는지
- dry-run에서 시리얼 포트를 열지 않고 Arduino로 보낼 CSV만 출력하는지
- RViz를 별도 실행했을 때 `/joint_states` → `/tf` 흐름으로 모델이 움직이는지
- 실제 Arduino에 보낼 때는 기본값인 `dry_run:=false`로 실행하는지
- 실행 사용자가 시리얼 장치 권한을 갖는지
- Arduino 보드레이트가 launch 값과 같은지
- 서보를 기구물에 연결하기 전에 최소·중립·최대 방향이 맞는지
- 그리퍼 0 ~ 8 mm가 실제 서보의 안전 범위와 맞는지

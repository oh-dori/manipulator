# manipulator

ROS 2 Humble용 소형 매니퓰레이터 리포지토리다. 이 리포 안에서 기본 로봇 패키지 `my_manipulator`와 MoveIt 설정 패키지 `my_manipulator_moveit`를 함께 관리한다.

`my_manipulator`는 하나의 공통 로봇 모델을 사용해 다음 세 가지 모드를 제공한다.

- RViz: 형상, TF, 조인트 범위를 수동으로 확인
- Gazebo Classic: `gazebo_ros2_control` 기반 물리 시뮬레이션
- 실제 로봇: Arduino와 서보모터를 사용하는 `ros2_control` 하드웨어 플러그인

## 로봇 구성

형상이 있는 링크는 `link_1`부터 `link_7`까지 7개다. 여기에 기준 좌표계인 `world` 링크가 하나 더 있다.

URDF에는 고정 조인트 1개와 가동 조인트 6개가 있다.

| 조인트 | 타입 | 범위 | 역할 |
|---|---|---:|---|
| `world_joint` | fixed | - | `world`와 베이스 고정 |
| `joint_1` | revolute | -90° ~ 90° | 베이스 회전 |
| `joint_2` | revolute | 0° ~ 180° | 어깨 |
| `joint_3` | revolute | -180° ~ 0° | 팔꿈치 |
| `joint_4` | revolute | -90° ~ 90° | 손목 |
| `joint_5_left` | prismatic | 0 ~ 8 mm | 그리퍼 명령축 |
| `joint_5_right` | prismatic mimic | 0 ~ 8 mm | 왼쪽 그리퍼를 따라가는 축 |

`joint_5_right`는 독립 명령을 받지 않는다. `joint_5_left`와 같은 위치값을 사용하지만 이동 축이 반대이므로 두 손가락이 대칭으로 열린다.

## 파일 구조

```text
manipulator/
├── README.md
├── LEARNING_NOTES.md
├── my_manipulator/
│   ├── config/my_controllers.yaml
│   ├── include/
│   ├── launch/
│   │   ├── display.launch.py
│   │   ├── gazebo.launch.py
│   │   └── real_robot.launch.py
│   ├── meshes/
│   ├── src/
│   ├── urdf/
│   │   ├── manipulator.xacro
│   │   ├── manipulator_sim.urdf.xacro
│   │   └── manipulator_real.urdf.xacro
│   ├── CMakeLists.txt
│   └── package.xml
└── my_manipulator_moveit/
    ├── config/
    │   └── moveit_controllers.yaml
    ├── launch/
    ├── rviz/
    ├── CMakeLists.txt
    └── package.xml
```

- `my_manipulator`: URDF/Xacro, mesh, ros2_control 설정, Gazebo/실제 로봇 launch, Arduino 하드웨어 인터페이스
- `my_manipulator_moveit`: MoveIt 2의 SRDF, kinematics, planning, controller 연결, MoveIt launch 설정을 둘 패키지
- `my_manipulator/urdf/manipulator.xacro`: 링크, 조인트, mesh 등 공통 형상
- `my_manipulator/urdf/manipulator_sim.urdf.xacro`: Gazebo 하드웨어와 mimic 설정 추가
- `my_manipulator/urdf/manipulator_real.urdf.xacro`: Arduino 하드웨어와 서보 보정값 추가
- `my_manipulator/config/my_controllers.yaml`: 5개 명령축을 제어하는 trajectory controller 설정
- `my_manipulator_moveit/config/moveit_controllers.yaml`: MoveIt이 `arm_controller`의 FollowJointTrajectory action을 찾기 위한 설정

## 설치와 빌드

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select my_manipulator my_manipulator_moveit
source install/setup.bash
```

## RViz 형상 확인

```bash
ros2 launch my_manipulator display.launch.py
```

`joint_state_publisher_gui`에는 `joint_1`부터 `joint_4`, `joint_5_left`까지 5개 슬라이더가 나타난다. `joint_5_right`는 mimic 조인트이므로 별도 슬라이더 없이 함께 움직인다.

RViz Fixed Frame은 `world`다.

실행 중인 Gazebo나 실제 로봇 상태를 보기만 할 때는 트랙바가 있는 `display.launch.py` 대신 RViz만 따로 실행한다.

```bash
rviz2 -d ~/ros2_ws/src/manipulator/my_manipulator/rviz/display.rviz
```

## Gazebo 시뮬레이션

터미널 1:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_manipulator gazebo.launch.py
```

필요하면 다른 터미널에서 RViz만 따로 실행해 `/joint_states`와 `/tf` 흐름을 확인한다.

```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/manipulator/my_manipulator/rviz/display.rviz
```

로봇 spawn이 완료된 후 터미널 2에서 컨트롤러 상태를 확인한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
```

예상 상태:

```text
joint_state_broadcaster       active
arm_controller                active
```

아래의 [조인트 이동 명령](#조인트-이동-명령)으로 로봇을 움직일 수 있다.

## 실제 Arduino 로봇

### Arduino 없이 디버그

Arduino를 연결하지 않고 하드웨어 인터페이스와 controller 동작을 확인한다.

터미널 1:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_manipulator real_robot.launch.py dry_run:=true
```

상태를 화면으로 확인하고 싶으면 다른 터미널에서 RViz만 따로 실행한다.

```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/manipulator/my_manipulator/rviz/display.rviz
```

이 모드에서는 시리얼 포트를 열지 않는다. 조인트 명령을 보내면 Arduino로
전송할 CSV가 다음 형태로 터미널 1에 출력된다.

```text
[dry-run] Arduino CSV: 135,90,90,90,60,1
```

터미널 2에서 컨트롤러 상태를 확인하고
[조인트 이동 명령](#조인트-이동-명령)을 전송한다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
```

### 실제 Arduino 연결

펌웨어의 `Serial.begin(9600)`과 동일하게 baud rate를 `9600`으로 사용한다.
포트 이름은 연결 환경에 맞게 변경한다.

터미널 1:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_manipulator real_robot.launch.py \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=9600 \
  write_rate_hz:=20
```

기본 포트 `/dev/ttyUSB0`을 사용한다면 인자 없이 실행할 수도 있다.

```bash
ros2 launch my_manipulator real_robot.launch.py
```

상태를 화면으로 확인하고 싶으면 RViz만 따로 실행한다. 실행 후 터미널 2에서 [조인트 이동 명령](#조인트-이동-명령)을 전송한다.

## 조인트 이동 명령

Gazebo, 실제 로봇 dry-run, 실제 Arduino에서 동일한 명령을 사용한다.
명령 대상은 5개다.

```bash
source ~/ros2_ws/install/setup.bash
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

회전 조인트 단위는 rad, 그리퍼 단위는 m다. 위 명령은 모든 조인트가
2초 후 지정한 위치에 도달하도록 controller에 요청한다.

원위치로 복귀:

```bash
ros2 topic pub --once \
  /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5_left"],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.008],
      time_from_start: {sec: 2, nanosec: 0}
    }]
  }'
```

### 시리얼 프로토콜

PC는 한 줄에 5개 서보 각도와 실행 방식을 CSV로 전송한다.

```text
90,90,90,90,30,1\n
```

순서는 다음과 같다.

```text
joint_1,joint_2,joint_3,joint_4,joint_5_left,move_immediately
```

`move_immediately`가 `1`이면 각도를 즉시 적용하고, `0`이면 Arduino가 정해진
시간 동안 목표 각도로 이동한다.

### 안전 동작

- 초기 명령은 각 조인트의 `initial_value`에서 시작한다.
- NaN이나 무한대 명령은 전송하지 않고 오류로 처리한다.
- URDF 조인트 범위를 벗어난 명령은 범위 안으로 제한한다.
- controller loop는 100 Hz지만 시리얼 출력은 기본 20 Hz로 제한한다.
- 현재 위치 센서가 없으므로 상태값은 마지막 명령을 현재 위치로 간주하는 open-loop 방식이다.

### 서보 보정

`manipulator_real.urdf.xacro`의 각 조인트에는 다음 값이 있다.

```xml
<param name="servo_min_angle">0</param>
<param name="servo_max_angle">180</param>
```

`command_interface`의 `min`은 `servo_min_angle`, `max`는 `servo_max_angle`에 선형 매핑된다. 서보 각도 범위는 실제 안전 각도 그대로 두고, ROS 조인트 방향과 서보 장착 방향이 반대인 조인트는 `command_interface`의 `min`/`max`를 서로 바꿔 보정한다.

실제 기구물에 연결하기 전에는 서보 혼만 분리하거나 부하가 없는 상태에서 각 조인트의 최소·최대 각도를 반드시 보정해야 한다.

## 검증 명령

```bash
cd ~/ros2_ws
colcon build --packages-select my_manipulator
colcon test --packages-select my_manipulator
colcon test-result --verbose

xacro src/manipulator/my_manipulator/urdf/manipulator.xacro > /tmp/manipulator.urdf
xacro src/manipulator/my_manipulator/urdf/manipulator_sim.urdf.xacro > /tmp/manipulator_sim.urdf
xacro src/manipulator/my_manipulator/urdf/manipulator_real.urdf.xacro > /tmp/manipulator_real.urdf
check_urdf /tmp/manipulator.urdf
check_urdf /tmp/manipulator_sim.urdf
check_urdf /tmp/manipulator_real.urdf
```

## 현재 한계

- 실제 관성값이 아니라 작은 근사값을 사용하므로 Gazebo 동역학 정확도는 제한적이다.
- 실제 로봇은 위치 피드백이 없는 open-loop 제어다.
- 서보 각도, 방향, 초기 자세는 실제 조립 상태에 맞춰 보정해야 한다.
- Gazebo Classic은 2025년 1월 지원 종료되었으므로 장기적으로 modern Gazebo와 `gz_ros2_control` 전환을 고려해야 한다.

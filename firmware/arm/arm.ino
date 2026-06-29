#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// 기존 펄스 범위: MIN 620us, MAX 2800us
#define SERVO_MIN_PULSE_US       620   // 서보 모터의 0도에 해당하는 최소 펄스 폭 (마이크로초)
#define SERVO_MAX_PULSE_US       2800  // 서보 모터의 180도에 해당하는 최대 펄스 폭 (마이크로초)
#define SERVO_PWM_FREQUENCY_HZ   50
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long last_status_log_ms = 0;
const unsigned long STATUS_LOG_INTERVAL_MS = 1000;

const unsigned long SERVO_UPDATE_INTERVAL_MS = 20;
const unsigned long MOVE_DURATION_MS = 3000;
unsigned long last_update_ms = 0;  // 마지막 서보 갱신 시각
unsigned long move_start_ms = 0;   // 현재 이동의 시작 시각
bool is_moving = false;

int joint_1 = 11;
int joint_2 = 12;
int joint_3 = 13;
int joint_4 = 14;
int joint_5 = 15;

int servo_channels[5] = {joint_1, joint_2, joint_3, joint_4, joint_5};
int current_angles[5] = {90, 0, 180, 90, 90};
int start_angles[5] = {90, 0, 180, 90, 90};
int target_angles[5] = {90, 0, 180, 90, 90};

void setup() 
{
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_PWM_FREQUENCY_HZ);

  delay(3000);

  setServoAngle(joint_1, 90);  // (-z) 0 ~ 90 ~ 180 (+z)
  setServoAngle(joint_2, 0);  // 0 ~ 180 (+y)
  setServoAngle(joint_3, 180);  // 180 ~ 0 (-y) 
  setServoAngle(joint_4, 90);  // (+y) 0 ~ 90 ~ 180 (-y) 
  setServoAngle(joint_5, 90);  //   30 ~ 90
}

void setServoAngle(int channel, int angle)
{
  // 각도를 마이크로초 단위의 펄스 폭으로 변환합니다.
  // 펄스 폭(us)을 PCA9685 컨트롤러가 사용하는 12비트 틱 값으로 변환합니다.
  // 1초 = 1,000,000 마이크로초
  // 주기(us) = 1,000,000 / 주파수(Hz)
  // 틱 = (펄스폭_us * 4096) / 주기_us
  long pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  long pulse_ticks =
      (pulse_us * 4096L) / (1000000L / SERVO_PWM_FREQUENCY_HZ);
  pwm.setPWM(channel, 0, pulse_ticks);
}

void updateServoPositions()
{
  if (!is_moving) {
    return;
  }

  unsigned long current_time = millis();
  if (current_time - last_update_ms < SERVO_UPDATE_INTERVAL_MS) {
    return;
  }
  last_update_ms = current_time;

  unsigned long elapsed_time = current_time - move_start_ms;

  if (elapsed_time >= MOVE_DURATION_MS) {
    for (int i = 0; i < 5; i++) {
      current_angles[i] = target_angles[i];
      setServoAngle(servo_channels[i], current_angles[i]);
    }
    is_moving = false;
    return;
  }

  for (int i = 0; i < 5; i++) {
    long angle_difference = (long)target_angles[i] - start_angles[i];
    int interpolated_angle =
        start_angles[i] +
        (angle_difference * (long)elapsed_time) / (long)MOVE_DURATION_MS;

    if (interpolated_angle != current_angles[i]) {
      current_angles[i] = interpolated_angle;
      setServoAngle(servo_channels[i], current_angles[i]);
    }
  }
}

void loop() 
{
  updateServoPositions();

  unsigned long current_time = millis();
  if (current_time - last_status_log_ms >= STATUS_LOG_INTERVAL_MS) {
    last_status_log_ms = current_time;
    Serial.println("Loop is running...");
  }

  // 시리얼 버퍼에 수신된 데이터가 있는지 확인합니다.
  if (Serial.available() > 0) {
    int angles[5];

    // 조인트 각도 5개와 즉시 실행 여부(0 또는 1)를 읽습니다.
    for (int i = 0; i < 5; i++) {
      angles[i] = Serial.parseInt();
    }
    int move_immediately = Serial.parseInt();

    // 데이터의 끝을 확인하기 위해 개행 문자가 들어올 때까지 기다립니다.
    if (Serial.read() == '\n') {
      if (move_immediately != 0 && move_immediately != 1) {
        Serial.println("Invalid move mode. Use 0 or 1.");
        return;
      }

      // 조인트 1~4는 0~180도, 조인트 5는 30~90도로 제한합니다.
      for (int i = 0; i < 4; i++) {
        angles[i] = constrain(angles[i], 0, 180);
      }
      angles[4] = constrain(angles[4], 30, 90);

      if (move_immediately == 1) {
        // ROS 2가 계산한 중간 목표값을 추가 보간 없이 바로 적용합니다.
        is_moving = false;
        for (int i = 0; i < 5; i++) {
          current_angles[i] = angles[i];
          setServoAngle(servo_channels[i], current_angles[i]);
        }
        return;
      }

      // 단독 제어에서는 현재 위치부터 목표 위치까지 정해진 시간 동안 이동합니다.
      for (int i = 0; i < 5; i++) {
        start_angles[i] = current_angles[i];
        target_angles[i] = angles[i];
      }
      move_start_ms = millis();
      last_update_ms = move_start_ms;
      is_moving = true;

      Serial.println("Timed move started.");
    }
  }
}

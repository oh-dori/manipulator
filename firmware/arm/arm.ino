#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// 기존 펄스 범위: MIN 620us, MAX 2800us
#define MIN_PULSE_WIDTH       620   // 서보 모터의 0도에 해당하는 최소 펄스 폭 (마이크로초)
#define MAX_PULSE_WIDTH       2800  // 서보 모터의 180도에 해당하는 최대 펄스 폭 (마이크로초)
#define FREQUENCY             50
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long previous_status_time = 0;
const unsigned long STATUS_INTERVAL_MS = 1000;

const unsigned long SERVO_UPDATE_INTERVAL_MS = 33;  // 90도 이동 시 약 3초
const int SERVO_STEP_DEGREES = 1;
unsigned long previous_servo_update_time = 0;

int joint_1 = 11;
int joint_2 = 12;
int joint_3 = 13;
int joint_4 = 14;
int joint_5 = 15;

int servo_channels[5] = {joint_1, joint_2, joint_3, joint_4, joint_5};
int current_angles[5] = {90, 0, 180, 90, 90};
int target_angles[5] = {90, 0, 180, 90, 90};

void setup() 
{
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  delay(3000); // Init position

  setServoAngle(joint_1, 90);  // (-z) 0 ~ 90 ~ 180 (+z)
  setServoAngle(joint_2, 0);  // 0 ~ 180 (+y)
  setServoAngle(joint_3, 180);  // 180 ~ 0 (-y) 
  setServoAngle(joint_4, 90);  // (+y) 0 ~ 90 ~ 180 (-y) 
  setServoAngle(joint_5, 90);  //   30 ~ 90
}

void setServoAngle(int channel, int angle)
{
  // 각도를 마이크로초 단위의 펄스 폭으로 변환합니다.
  long pulse_us = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  // 펄스 폭(us)을 PCA9685 컨트롤러가 사용하는 12비트 틱 값으로 변환합니다.
  // 1초 = 1,000,000 마이크로초
  // 주기(us) = 1,000,000 / 주파수(Hz)
  // 틱 = (펄스폭_us * 4096) / 주기_us
  long pulse_ticks = (pulse_us * 4096L) / (1000000L / FREQUENCY);
  pwm.setPWM(channel, 0, pulse_ticks);
}

void updateServoPositions()
{
  unsigned long current_time = millis();
  if (current_time - previous_servo_update_time < SERVO_UPDATE_INTERVAL_MS) {
    return;
  }
  previous_servo_update_time = current_time;

  for (int i = 0; i < 5; i++) {
    if (current_angles[i] < target_angles[i]) {
      current_angles[i] = min(current_angles[i] + SERVO_STEP_DEGREES, target_angles[i]);
      setServoAngle(servo_channels[i], current_angles[i]);
    } else if (current_angles[i] > target_angles[i]) {
      current_angles[i] = max(current_angles[i] - SERVO_STEP_DEGREES, target_angles[i]);
      setServoAngle(servo_channels[i], current_angles[i]);
    }
  }
}

void loop() 
{
  updateServoPositions();

  unsigned long current_time = millis();
  if (current_time - previous_status_time >= STATUS_INTERVAL_MS) {
    previous_status_time = current_time;
    Serial.println("Loop is running...");
  }

  // 시리얼 버퍼에 수신된 데이터가 있는지 확인합니다.
  if (Serial.available() > 0) {
    // 각도 값을 저장할 변수를 선언합니다.
    int angle1, angle2, angle3, angle4, angle5;

    // Serial.parseInt()로 정수를 순서대로 읽어옵니다.
    angle1 = Serial.parseInt();
    angle2 = Serial.parseInt();
    angle3 = Serial.parseInt();
    angle4 = Serial.parseInt();
    angle5 = Serial.parseInt();

    // 데이터의 끝을 확인하기 위해 개행 문자가 들어올 때까지 기다립니다.
    if (Serial.read() == '\n') {
      
      // -- 여기부터 수정된 부분 --
      // constrain(값, 최소, 최대) 함수로 각도 범위를 0~180으로 제한합니다.
      angle1 = constrain(angle1, 0, 180);
      angle2 = constrain(angle2, 0, 180);
      angle3 = constrain(angle3, 0, 180);
      angle4 = constrain(angle4, 0, 180);
      angle5 = constrain(angle5, 30, 90);
      // -- 여기까지 수정된 부분 --

      // 제한된 각도 값을 터미널에 출력하여 확인합니다.
      Serial.print("Setting constrained angles to: ");
      Serial.print(angle1); Serial.print(", ");
      Serial.print(angle2); Serial.print(", ");
      Serial.print(angle3); Serial.print(", ");
      Serial.print(angle4); Serial.print(", ");
      Serial.println(angle5);

      // 현재 위치에서 목표 위치까지 일정한 간격으로 부드럽게 이동합니다.
      target_angles[0] = angle1;
      target_angles[1] = angle2;
      target_angles[2] = angle3;
      target_angles[3] = angle4;
      target_angles[4] = angle5;
    }
  }
}

// Include required headers
#include <tle94112-ino.hpp>
#include "Arduino_BMI270_BMM150.h"
#include "Kalman.h"

Kalman kalmanY;
// -------------------------- 1. PID Parameters and Variables --------------------------
float Kp = 2.5, Ki = 0.1, Kd = 0.08;
float setpoint = 0.6, previous_error = 0.0, integral = 0.0;
float accAngle, gyroRate, angle, dt = 0.01;
float errorSum = 0, lastError = 0;
unsigned long lastTime = 0;
int16_t output = 0;

// -------------------------- Movement Control State --------------------------
enum MoveState { STOP, FWD, BACK, LEFT, RIGHT };
MoveState moveState = STOP;
float target_setpoint = 0.0, turnOffset = 0.0;
bool hasPushed = false, debugEnabled = true;

// -------------------------- TLE Motor Control --------------------------
uint8_t motorReg[2][2] = {{REG_ACT_1, REG_PWM_DC_1}, {REG_ACT_3, REG_PWM_DC_3}};
volatile uint8_t oldDirection[] = { LL_HH, HH_LL };
Tle94112Ino controller = Tle94112Ino(3,4);

// -------------------------- Global IMU Data --------------------------


// -------------------------- loop Timing --------------------------
unsigned long loopStartTime = 0;

// -------------------------- Motor Control --------------------------
void motorSet(uint8_t motorNum, uint8_t dir, uint8_t speed, bool errorCheck = false) {
    if (dir != oldDirection[motorNum]) {
        controller.directWriteReg(motorReg[motorNum][0], dir);
        oldDirection[motorNum] = dir;
    }
    controller.directWriteReg(motorReg[motorNum][1], speed);
    if (errorCheck) controller.clearErrors();
}

void motor_pwm(int motor, int speed) {
    uint8_t dirForward  = (motor == 0) ? LL_HH : HH_LL;
    uint8_t dirBackward = (motor == 0) ? HH_LL : LL_HH;
    if (speed > 0) motorSet(motor, dirForward, abs(speed));
    else if (speed < 0) motorSet(motor, dirBackward, abs(speed));
    else motorSet(motor, oldDirection[motor], 0);
}

void driveMotors(int16_t baseOutput, float turnOffset) {
    int16_t left = constrain(baseOutput + turnOffset, -100, 100);
    int16_t right = constrain(baseOutput - turnOffset, -100, 100);
    motor_pwm(0, left);
    motor_pwm(1, right);
}

int16_t motor_ramp(int16_t motor_in) {
    if (abs(motor_in) < 3) return 0;
    return (motor_in > 0) ? motor_in + 20 : motor_in - 20;
}

// -------------------------- initialisation --------------------------
void setMultiHalfbridge(bool state) {
    digitalWrite(4, state ? HIGH : LOW);
    if (state) {
        controller.begin();
        controller.configHB(controller.TLE_HB1, controller.TLE_HIGH, controller.TLE_PWM1);
        controller.configHB(controller.TLE_HB2, controller.TLE_HIGH, controller.TLE_PWM1);
        controller.configHB(controller.TLE_HB3, controller.TLE_LOW,  controller.TLE_NOPWM);
        controller.configHB(controller.TLE_HB4, controller.TLE_LOW,  controller.TLE_NOPWM);
        controller.configHB(controller.TLE_HB9,  controller.TLE_HIGH, controller.TLE_PWM3);
        controller.configHB(controller.TLE_HB10, controller.TLE_HIGH, controller.TLE_PWM3);
        controller.configHB(controller.TLE_HB11, controller.TLE_LOW,  controller.TLE_NOPWM);
        controller.configHB(controller.TLE_HB12, controller.TLE_LOW,  controller.TLE_NOPWM);
        controller.configPWM(controller.TLE_PWM1, controller.TLE_FREQ200HZ, 0);
        controller.configPWM(controller.TLE_PWM3, controller.TLE_FREQ200HZ, 0);
        controller.clearErrors();
    }
}

void setup() {
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    pinMode(LED2, OUTPUT);
    Serial.begin(9600);
    while (!Serial);
    controller.begin();
    setMultiHalfbridge(true);
    if (!IMU.begin()) while (1);
    lastTime = millis();
    Serial.println("Started");
}

// -------------------------- Control Logic Functions --------------------------
void readIMUData() {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0;
    lastTime = now;
    accAngle = atan2(ax, az) * 180 / PI;
    kalmanY.getAngle(accAngle,gy,dt);
    gyroRate = gy;
    angle = accAngle; // Use acceleration angle directly as current angle, no filter control logic function
}

void updateMovementControl(float Gyroz) {
    switch (moveState) {
        case FWD:
        case BACK:
            target_setpoint = hasPushed ? target_setpoint * 0.96 : ((moveState == FWD) ? 1.5 : -1.5);
            hasPushed = true;
            if (abs(target_setpoint) < 0.1) {
                target_setpoint = 0;
                moveState = STOP;
                hasPushed = false;
            }
            turnOffset = 0;
            break;
        case LEFT:
            target_setpoint = 0;
            turnOffset = +30 - 0.5 * Gyroz;
            break;
        case RIGHT:
            target_setpoint = 0;
            turnOffset = -30 - 0.5 * Gyroz;
            break;
        case STOP:
        default:
            target_setpoint = 0;
            turnOffset = 0;
            hasPushed = false;
            break;
    }
}

void computePID() {
    setpoint = 0.9 * setpoint + 0.1 * target_setpoint;
    float error = setpoint - angle;
    errorSum += error * dt;
    errorSum = constrain(errorSum, -20.0, 20.0);
    float dError = (error - lastError) / dt;
    float pidOut = Kp * error + Ki * errorSum + Kd * dError;
    lastError = error;
    output = constrain(round(motor_ramp(pidOut)), -100, 100);
    if (abs(output) < 15) output = 0;
    output = -output;
}

void debugPrint(unsigned long loopDuration) {
    if (debugEnabled) {
        Serial.print("Angle: "); Serial.print(angle);
        //Serial.print(" | Gy: ");  Serial.print(gy, 2);
        Serial.print(" | Out: "); Serial.print(output);
        //Serial.print(" | Target: "); Serial.print(target_setpoint);
        Serial.print(" | KP:");Serial.println(Kp);
        Serial.print(" | Ki:");Serial.println(Ki);
        Serial.print(" | Kd:");Serial.println(Kd);
        Serial.print(" | Mode: ");
        switch (moveState) {
            case STOP: Serial.print("STOP"); break;
            case FWD: Serial.print("FWD"); break;
            case BACK: Serial.print("BACK"); break;
            case LEFT: Serial.print("LEFT"); break;
            case RIGHT: Serial.print("RIGHT"); break;
        }
        Serial.print(" | Loop time: ");
        Serial.print(loopDuration);
        Serial.println(" ms");
    }
}

// -------------------------- Serial Input Processing --------------------------
void checkSerialInput() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("kp ") || input.startsWith("ki ") || input.startsWith("kd ")) {
        errorSum = lastError = 0;
    }
    if (input.startsWith("kp ")) Kp = input.substring(3).toFloat();
    else if (input.startsWith("ki ")) Ki = input.substring(3).toFloat();
    else if (input.startsWith("kd ")) Kd = input.substring(3).toFloat();
    else if (input.startsWith("d ")) debugEnabled = (input.substring(2).toInt() == 1);
    else if (input == "fwd") moveState = FWD;
    else if (input == "back") moveState = BACK;
    else if (input == "left") moveState = LEFT;
    else if (input == "right") moveState = RIGHT;
    else if (input == "stop") moveState = STOP;
    else if (input == "PING") Serial.println("PONG");
    else if (input == "bridge on") setMultiHalfbridge(true);
    else if (input == "bridge off") setMultiHalfbridge(false);
}

// -------------------------- Main Loop --------------------------
void loop() {
    loopStartTime = millis();
    checkSerialInput();
    if (IMU.accelerationAvailable()) 
    {
        readIMUData();
        updateMovementControl(gy);
        computePID();
        driveMotors(output, turnOffset);
        unsigned long loopDuration = millis() - loopStartTime;
        debugPrint(loopDuration);

    }
}

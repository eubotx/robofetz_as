#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <ESP32Encoder.h>
#include <algorithm> // For std::sort

// Debug macros (uncomment to enable)
#define DEBUG_DRIVE_LEFT    // Enable left debug topic
#define DEBUG_DRIVE_RIGHT   // Enable right debug topic
//#define DEBUG_LOOPTIME             // Enable loop time debug topic


// Robot parameters
constexpr double WHEEL_DIAMETER = 0.069;
constexpr double WHEEL_DISTANCE = 0.19;
constexpr double MAX_MOTOR_SPEED = 180.0;  // Max speed in RPS
constexpr int MAX_MOTOR_DRIVER_DUTYCYCLE = 190; //255*(3/4) because we are running 4S Lipo
constexpr int ENCODER_TICKS_PER_REVOLUTION = 14 * 2;
constexpr double ENCODER_PT1_TIMECONSTANT_S = 0.1;  //TODO
constexpr double GEARBOX_RATIO = 1.0/200.0;
constexpr int MAX_WEAPON_MOTOR_DRIVER_DUTYCYCLE = 50;

constexpr unsigned long UPDATE_INTERVAL_ENCODER = 1;  // In ms
constexpr unsigned long UPDATE_INTERVAL_PID_CONTROL = 1;  // In ms

// PID controller parameters - adjust these during tuning
double KP = 1.0;
double KI = 3.0;
double KD = 0.0;
double KI_MAX = 20.0;

// WiFi configuration
//constexpr const char* MY_IP = "192.168.8.171"; // Bjoern Gelb DO NOT DELETE
//constexpr const char* MY_IP = "192.168.8.204"; // Flo Gelb DO NOT DELETE
constexpr const char* MY_IP = "192.168.8.231"; // Dario Gelb DO NOT DELETE
constexpr const char* MY_SSID = "GL-MT300N-V2-7d8";
constexpr const char* MY_PASSWORD = "goodlife";
constexpr int MY_PORT = 8888;

// Pin definitions
constexpr int LEFT_ENCODER_C2_PIN = 18;
constexpr int LEFT_ENCODER_C1_PIN = 19;
constexpr int RIGHT_ENCODER_C1_PIN = 5;
constexpr int RIGHT_ENCODER_C2_PIN = 4;

constexpr int MOTORDRIVER_LEFT_CW_PIN = 32;
constexpr int MOTORDRIVER_LEFT_CCW_PIN = 16;
constexpr int MOTORDRIVER_LEFT_PWM_PIN = 17;

constexpr int MOTORDRIVER_RIGHT_CW_PIN = 25;
constexpr int MOTORDRIVER_RIGHT_CCW_PIN = 26;
constexpr int MOTORDRIVER_RIGHT_PWM_PIN = 27;

constexpr int MOTORDRIVER_STBY_PIN = 33;

constexpr int MOTORDRIVER_WEAPON_PIN = 23;

// 4 Floating


// ROS2 entities

#ifdef DEBUG_DRIVE_LEFT
rcl_publisher_t debugDriveLeftPublisher;
std_msgs__msg__Float64MultiArray debugMotorcontrolLeftMsg;
#endif

#ifdef DEBUG_DRIVE_RIGHT
rcl_publisher_t debugDriveRightPublisher; 
std_msgs__msg__Float64MultiArray debugMotorcontrolRightMsg;
#endif

#ifdef DEBUG_LOOPTIME
rcl_publisher_t debugLooptimePublisher;
std_msgs__msg__Float64MultiArray debugLooptimeMsg;
#endif

rcl_subscription_t twistSubscriber;
rcl_subscription_t pidTuningSubscriber;
rcl_subscription_t weaponSpeedSubscriber;
std_msgs__msg__Float64MultiArray pidTuningMsg;
geometry_msgs__msg__Twist twistMsg;
std_msgs__msg__Float32 weaponSpeedMsg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) {} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) {} }

// Encoder class
class Encoder {
public:
    Encoder(int pin1, int pin2, int ticksPerRevolution, float timeConstant, unsigned long updateInterval)
        : pin1(pin1), pin2(pin2), ticksPerRevolution(ticksPerRevolution), timeConstant(timeConstant), updateInterval(updateInterval*1000) {}
    
    void init() {
        pinMode(pin1, INPUT_PULLUP);
        pinMode(pin2, INPUT_PULLUP);

        // Enable the weak pull up resistors
        ESP32Encoder::useInternalWeakPullResistors = puType::up;

        encoder.attachHalfQuad(pin1, pin2);
        encoder.setCount(0);
        
        lastUpdateTime = micros();
    }

    void update() {
        unsigned long currentTime = micros();

        unsigned long deltaTime = currentTime - lastUpdateTime;
        
        if (updateInterval == 0 || deltaTime >= updateInterval) {
            
            long currentCount = encoder.getCount();
            
            // Calculate raw speed
            
            if (deltaTime > 0) {  // Prevent division by zero
                double rawRps = (currentCount - lastCount) / (ticksPerRevolution * (deltaTime / 1000000.0));
                
                // Apply PT1 filter if time constant is positive
                if (timeConstant > 0.0) {
                    double alpha = deltaTime / (timeConstant * 1000000.0 + deltaTime);
                    filteredRps = alpha * rawRps + (1.0 - alpha) * filteredRps;
                } else {
                    // No filtering if time constant is zero or negative
                    filteredRps = rawRps;
                }
            }
            
            lastCount = currentCount;
            lastUpdateTime = currentTime;
        }
    }

    void resetCount() {
        encoder.setCount(0);
        lastCount = 0;
    }

    double getSpeed() const {
        return filteredRps;
    }

    double getAngularPosition() const {
        return (lastCount / (double)ticksPerRevolution);
    }

    void setTimeConstant(float tc) {
        timeConstant = tc;
    }

    void setUpdateInterval(unsigned long interval) {
        updateInterval = interval * 1000;
    }

private:
    ESP32Encoder encoder;
    int pin1, pin2;
    int ticksPerRevolution;
    long lastCount = 0;
    unsigned long lastUpdateTime = 0;
    unsigned long updateInterval;
    double filteredRps = 0.0;
    double timeConstant;
};

// MotorDriver class
class MotorDriver {
public:
    MotorDriver(int pin1, int pin2, int pin3, const int maxDutyCycle, int channel, int freq = 5000, int resolution = 8) : 
        pin1(pin1), pin2(pin2), pin3(pin3), 
        maxDutyCycle(maxDutyCycle),
        channel(channel),
        freq(freq),
        resolution(resolution) {}

    void init() {
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        
        // Configure LEDC channel
        ledcSetup(channel, freq, resolution);
        ledcAttachPin(pin3, channel);
    }

    void update() {
        if (dutyCycle > 0) {
            digitalWrite(pin2, LOW);
            digitalWrite(pin1, HIGH);
            ledcWrite(channel, abs(dutyCycle));
        } else if (dutyCycle < 0) {
            digitalWrite(pin1, LOW);
            digitalWrite(pin2, HIGH);
            ledcWrite(channel, abs(dutyCycle));
        } else {
            ledcWrite(channel, 0);
        }
    }

    void stop() {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        ledcWrite(channel, 0);
    }

    void setMotorDutyCycle(int dutyCycle) {
        this->dutyCycle = (int)constrain(dutyCycle, -maxDutyCycle, maxDutyCycle);
    }

    int getMotorDutyCycle() const { return dutyCycle; }

private:
    int pin1, pin2, pin3;
    int dutyCycle = 0;
    int maxDutyCycle;
    int channel;
    int freq;
    int resolution;
};

// PIDController class
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, double KiMax) 
        : Kp(Kp), Ki(Ki), Kd(Kd), KiMax(KiMax), error(0), integral(0), derivative(0), previousError(0) {
          lastTime = micros();
          }

    double compute(double desiredValue, double measuredValue) {
        double deltaTime = (micros() - lastTime) / 1000000.0;
        error = desiredValue - measuredValue;
        integral = integral + error*deltaTime;
        integral = constrain(integral, -KiMax, KiMax);  // Anti-windup
        derivative = (error - previousError)/deltaTime;
        previousError = error;
        lastTime = micros();

        return (Kp * error + Ki * integral + Kd * derivative);
    }

    void pause(){
      lastTime = micros();
    }

    void pidReset() {
        error = 0.0;
        integral = 0.0;
        derivative = 0.0;
        previousError = 0.0;
        lastTime = micros();
    }

    void setPidValues(double Kp, double Ki, double Kd, double KiMax) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->KiMax = KiMax;
        pidReset();
    }

    double getKp() const { return Kp; }

    double getKi() const {return Ki; }

    double getKd() const { return Kd; }

    double getKiMax() const { return KiMax; }

    double getError() const { return error; }

    double getIntegral() const { return integral; }

    double getDerivative() const { return derivative; }

    double getPreviousError() const { return previousError; }

private:
    double Kp, Ki, Kd, KiMax;
    double error, integral, derivative, previousError;
    long lastTime = 0;
};

class PidMotor {
public:

    PidMotor(Encoder& encoder, MotorDriver& motorDriver, PIDController& pid, unsigned int controllCycleTime)
        : encoder(encoder), motorDriver(motorDriver), pid(pid), controllCycleTime(controllCycleTime) {}

    void init(){
      encoder.init();
      motorDriver.init();
    }
    
    void update() {
        
        encoder.update(); //TODO das besser organisieren außerhalb der Klasse
        
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= controllCycleTime) {

            lastUpdateTime = currentTime;

            if (desiredMotorSpeed == 0.0){
              motorTimeoutThresh ++;
            } else {
              motorTimeoutThresh = 0;
            }
            
            if (motorTimeoutThresh > 100) {   // equals 100 * controllCycleTime until motors go to sleep
              motorDriver.stop();
              motorDriver.setMotorDutyCycle(0);
              pid.pidReset();
            } else {
              int pidValue = int(round(pid.compute(desiredMotorSpeed, encoder.getSpeed()))); // Apply PID control   //TODO ist round wirklich auf vor dezimal?
              motorDriver.setMotorDutyCycle(pidValue); // Set motor speed based on PWM
              motorDriver.update();
            }
        }
    }

    void setDesiredMotorSpeed(double desiredMotorSpeed) {
        this->desiredMotorSpeed = desiredMotorSpeed;
    }
    
    double getDesiredMotorSpeed() const {
        return desiredMotorSpeed;
    }

    double getMeasuredMotorSpeed() const {
        return encoder.getSpeed();
    }

    double getAngularMotorPosition() {
        return encoder.getAngularPosition();
    }

private:
    Encoder& encoder;
    MotorDriver& motorDriver;
    PIDController& pid;
    double desiredMotorSpeed = 0; // Desired motor speed (RPS)
    unsigned long controllCycleTime;
    unsigned long lastUpdateTime = 0;
    unsigned long motorTimeoutThresh = 0;
};

class Wheel {
public:
    Wheel(PidMotor& pidMotor, double gearboxRatio)
        : pidMotor(pidMotor), gearboxRatio(gearboxRatio) {}

    void init(){
      pidMotor.init();
    }
    
    void update() {
        pidMotor.update();
    }

    void setDesiredWheelSpeed(double desiredWheelSpeed) {
        this->desiredWheelSpeed = desiredWheelSpeed; // Set wheel speed
        // Convert wheel speed to motor speed using the gearbox ratio
        pidMotor.setDesiredMotorSpeed(desiredWheelSpeed / gearboxRatio);
    }

    double getDesiredWheelSpeed() const {
        return desiredWheelSpeed;
    }

    double getMeasuredWheelSpeed() {
        return pidMotor.getMeasuredMotorSpeed() * gearboxRatio;
    }

    double getAngularPosition() {
        return pidMotor.getAngularMotorPosition() * gearboxRatio;
    }

private:
    PidMotor& pidMotor;
    double desiredWheelSpeed = 0.0; // Desired wheel speed (RPS)
    double angularWheelPosition = 0.0;
    double gearboxRatio; // Gearbox ratio to convert wheel speed to motor speed
};

// Weapon class
class Weapon {
public:
    Weapon(int pwmPin, int channel, int pwmFreq = 50, int pwmResolution = 12,
           int pwmMin = 205, int pwmMax = 410, int pwmArm = 184)
        : pinPwm(pwmPin),
          channel(channel),
          freq(pwmFreq),
          resolution(pwmResolution),
          PWM_MIN(pwmMin),
          PWM_MAX(pwmMax),
          PWM_ARM(pwmArm),
          maxDutyCycle(pwmMax) {
    }

    void setWeaponSpeed(double speed) {
        weaponSpeed = constrain(speed, 0.0, 1.0);
        motorControllerDutyCycle = mapFloat(weaponSpeed, 0.0, 1.0, PWM_MIN, PWM_MAX);
    }

    void init() {
        // Configure PWM with LEDC
        ledcSetup(channel, freq, resolution);
        ledcAttachPin(pinPwm, channel);
        
        // Execute arming sequence
        performArmingSequence();
    }

    void update() {
        ledcWrite(channel, motorControllerDutyCycle);
    }

    void stop() {
        ledcWrite(channel, PWM_MIN);  // Send minimum throttle
    }

    double getCurrentSpeed() const {
        return weaponSpeed;
    }

private:
    // Configuration parameters (set during construction)
    const int pinPwm;
    const int channel;
    const int freq;
    const int resolution;
    const int PWM_MIN;
    const int PWM_MAX;
    const int PWM_ARM;
    const int maxDutyCycle;
    
    // Runtime variables
    double weaponSpeed = 0.0;
    int motorControllerDutyCycle = PWM_MIN;  // Start at minimum throttle

    void performArmingSequence() {
        
        // 1. Send low throttle
        ledcWrite(channel, PWM_MIN);
        delay(100);
        
        // 2. Send arming pulse
        ledcWrite(channel, PWM_ARM);
        delay(100);
        
        // 3. Return to low throttle
        ledcWrite(channel, PWM_MIN);
        delay(100);
    }

    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

// RobotControl Class
class RobotControl {
public:
    RobotControl(Wheel& leftWheel, Wheel& rightWheel, Weapon& weapon, double wheelDiameter, double wheelDistance)
        : leftWheel(leftWheel), rightWheel(rightWheel), weapon(weapon), wheelDiameter(wheelDiameter), wheelRadius(wheelDiameter/2.), wheelDistance(wheelDistance) {}

    void init(){
      leftWheel.init();
      rightWheel.init();
      weapon.init();
    }
    
    void calculateAndSetWheelSpeeds(double linear, double angular) {
        double linearWheelSpeed = linear;
        double angularWheelSpeed = angular * (wheelDistance / 2.0);
        
        double leftWheelSpeed = linearWheelSpeed + angularWheelSpeed;
        double rightWheelSpeed = linearWheelSpeed - angularWheelSpeed;

        // Convert the wheel speeds from m/s to rps
        double leftWheelRps = leftWheelSpeed / (M_PI * wheelDiameter);
        double rightWheelRps = rightWheelSpeed / (M_PI * wheelDiameter);
        
        leftWheel.setDesiredWheelSpeed(leftWheelRps);
        rightWheel.setDesiredWheelSpeed(rightWheelRps);
    }

    void updateDrives() {
        leftWheel.update();
        rightWheel.update();
    }

    void updateWeapon() {
        weapon.update();
    }

    private:
        Wheel& leftWheel;
        Wheel& rightWheel;
        Weapon& weapon;
        const double wheelRadius;
        const double wheelDiameter;
        const double wheelDistance;

};

#ifdef DEBUG_LOOPTIME
// Loop time variables
constexpr int LOOPTIME_WINDOW_SIZE = 10; // Number of cycles to track
unsigned long loopTimes[LOOPTIME_WINDOW_SIZE] = {0}; // Array to store loop times
int loopTimeIndex = 0; // Index for the current loop time
unsigned long lastLoopTime = 0; // Time of the last loop iteration


// Function to calculate loop time statistics
void collectLooptime() {
    unsigned long currentTime = micros();
    unsigned long loopTime = currentTime - lastLoopTime;
    lastLoopTime = currentTime;

    // Store the current loop time in the array
    loopTimes[loopTimeIndex] = loopTime;
    loopTimeIndex = (loopTimeIndex + 1) % LOOPTIME_WINDOW_SIZE;
}
#endif

// Initialize encoders
Encoder leftEncoder(LEFT_ENCODER_C1_PIN, LEFT_ENCODER_C2_PIN, ENCODER_TICKS_PER_REVOLUTION, ENCODER_PT1_TIMECONSTANT_S, UPDATE_INTERVAL_ENCODER);
Encoder rightEncoder(RIGHT_ENCODER_C1_PIN, RIGHT_ENCODER_C2_PIN, ENCODER_TICKS_PER_REVOLUTION,ENCODER_PT1_TIMECONSTANT_S, UPDATE_INTERVAL_ENCODER);

// Initialize motor controllers
MotorDriver leftMotorDriver(MOTORDRIVER_LEFT_CW_PIN, MOTORDRIVER_LEFT_CCW_PIN, MOTORDRIVER_LEFT_PWM_PIN,  MAX_MOTOR_DRIVER_DUTYCYCLE, 0);
MotorDriver rightMotorDriver(MOTORDRIVER_RIGHT_CW_PIN, MOTORDRIVER_RIGHT_CCW_PIN, MOTORDRIVER_RIGHT_PWM_PIN, MAX_MOTOR_DRIVER_DUTYCYCLE, 1);

// Initialize PID controllers
PIDController leftPid(KP, KI, KD, KI_MAX);
PIDController rightPid(KP, KI, KD, KI_MAX);

// Initialize motors
PidMotor leftMotor(leftEncoder, leftMotorDriver, leftPid, UPDATE_INTERVAL_PID_CONTROL);
PidMotor rightMotor(rightEncoder, rightMotorDriver, rightPid, UPDATE_INTERVAL_PID_CONTROL);

// Initialize wheels
Wheel leftWheel(leftMotor, GEARBOX_RATIO);
Wheel rightWheel(rightMotor, GEARBOX_RATIO);

// Initialize Weapon
Weapon weapon(MOTORDRIVER_WEAPON_PIN, 14);

// Initialize robot control
RobotControl robot(leftWheel, rightWheel, weapon, WHEEL_DIAMETER, WHEEL_DISTANCE);


// Timer callback
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {

#ifdef DEBUG_DRIVE_LEFT
        // Publish left debug information (PID + PWM)
        debugMotorcontrolLeftMsg.data.data[0] = leftMotor.getDesiredMotorSpeed();  // Desired value
        debugMotorcontrolLeftMsg.data.data[1] = leftMotor.getMeasuredMotorSpeed(); // Measured value
        debugMotorcontrolLeftMsg.data.data[2] = leftMotor.getDesiredMotorSpeed() - leftMotor.getMeasuredMotorSpeed(); // Error
        debugMotorcontrolLeftMsg.data.data[3] = leftMotorDriver.getMotorDutyCycle(); // PWM duty cycle
        RCSOFTCHECK(rcl_publish(&debugDriveLeftPublisher, &debugMotorcontrolLeftMsg, NULL));
#endif

#ifdef DEBUG_DRIVE_RIGHT
        // Publish right debug information (PID + PWM)
        debugMotorcontrolRightMsg.data.data[0] = rightMotor.getDesiredMotorSpeed();  // Desired value
        debugMotorcontrolRightMsg.data.data[1] = rightMotor.getMeasuredMotorSpeed(); // Measured value
        debugMotorcontrolRightMsg.data.data[2] = rightMotor.getDesiredMotorSpeed() - rightMotor.getMeasuredMotorSpeed(); // Error
        debugMotorcontrolRightMsg.data.data[3] = rightMotorDriver.getMotorDutyCycle(); // PWM duty cycle
        RCSOFTCHECK(rcl_publish(&debugDriveRightPublisher, &debugMotorcontrolRightMsg, NULL));
#endif

#ifdef DEBUG_LOOPTIME
        // Calculate statistics
        unsigned long sortedLoopTimes[LOOPTIME_WINDOW_SIZE];
        memcpy(sortedLoopTimes, loopTimes, sizeof(loopTimes));
        std::sort(sortedLoopTimes, sortedLoopTimes + LOOPTIME_WINDOW_SIZE);

        unsigned long minLoopTime = sortedLoopTimes[0];
        unsigned long maxLoopTime = sortedLoopTimes[LOOPTIME_WINDOW_SIZE - 1];
        unsigned long medianLoopTime = sortedLoopTimes[LOOPTIME_WINDOW_SIZE / 2];
        
        // Publish loop time statistics
        
        debugLooptimeMsg.data.data[0] = medianLoopTime;  // Median
        debugLooptimeMsg.data.data[1] = minLoopTime;     // Minrclc_subscription_init_default
        debugLooptimeMsg.data.data[2] = maxLoopTime;     // Max
        RCSOFTCHECK(rcl_publish(&debugLooptimePublisher, &debugLooptimeMsg, NULL));
#endif

    }
}

// Twist message callback
void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    robot.calculateAndSetWheelSpeeds(msg->linear.x, msg->angular.z);
}

// PID tuning callback
void pid_tuning_callback(const void* msgin) {
    const std_msgs__msg__Float64MultiArray* msg = (const std_msgs__msg__Float64MultiArray*)msgin;

    // Ensure the message contains exactly 4 values (Kp, Ki, Kd, KiMax)
    
    if (msg->data.size == 4) {
        double Kp = msg->data.data[0];
        double Ki = msg->data.data[1];
        double Kd = msg->data.data[2];
        double KiMax = msg->data.data[3];

        // Update PID parameters for both left and right controllers
        leftPid.setPidValues(Kp, Ki, Kd, KiMax);
        rightPid.setPidValues(Kp, Ki, Kd, KiMax);
    }
}


// PID tuning callback
void weapon_speed_callback(const void* msgin) {
    const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;

    double weaponSpeed = msg->data;

    weapon.setWeaponSpeed(weaponSpeed);
}


void setup() {
    set_microros_wifi_transports((char*)MY_SSID, (char*)MY_PASSWORD, (char*)MY_IP, MY_PORT);

    pinMode(MOTORDRIVER_STBY_PIN, OUTPUT);
    digitalWrite(MOTORDRIVER_STBY_PIN, HIGH);
    robot.init();

    //Serial.begin(9600);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "diff_drive_bot_esp32", "", &support));
    RCCHECK(rclc_subscription_init_default(&twistSubscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));  //TODO change to best effort?
    RCCHECK(rclc_subscription_init_default(&pidTuningSubscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "drive/pid_tuning"));  //TODO change to parameter?
    RCCHECK(rclc_subscription_init_default(&weaponSpeedSubscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "weapon/speed"));
    pidTuningMsg.data.capacity = 4;  // 4 values for the left and right PID tuning (Kp, Ki, Kd, KiMax)
    pidTuningMsg.data.size = 4;      // 8 values total (4 for left, 4 for right)
    pidTuningMsg.data.data = (double*)malloc(4 * sizeof(double)); // Allocate memory for 8 values
    weaponSpeedMsg.data = 0.0;  // 1 value

#ifdef DEBUG_DRIVE_LEFT
    // Initialize left debug publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &debugDriveLeftPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "drive/left/debug"
    ));
    debugMotorcontrolLeftMsg.data.capacity = 4;
    debugMotorcontrolLeftMsg.data.size = 4;
    debugMotorcontrolLeftMsg.data.data = (double*)malloc(4 * sizeof(double));
#endif

#ifdef DEBUG_DRIVE_RIGHT
    // Initialize right debug publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &debugDriveRightPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "drive/right/debug"
    ));
    debugMotorcontrolRightMsg.data.capacity = 4;
    debugMotorcontrolRightMsg.data.size = 4;
    debugMotorcontrolRightMsg.data.data = (double*)malloc(4 * sizeof(double));
#endif

#ifdef DEBUG_LOOPTIME
    // Initialize loop time debug publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &debugLooptimePublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "looptime_debug"
    ));
    debugLooptimeMsg.data.capacity = 3;
    debugLooptimeMsg.data.size = 3;
    debugLooptimeMsg.data.data = (double*)malloc(3 * sizeof(double));
#endif

    const unsigned int timer_period_ms = 100;
    RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_period_ms), timer_callback, true));
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &twistSubscriber, &twistMsg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &pidTuningSubscriber, &pidTuningMsg, &pid_tuning_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &weaponSpeedSubscriber, &weaponSpeedMsg, &weapon_speed_callback, ON_NEW_DATA));
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000)));  //max time budget 1000 ms 

    robot.updateDrives();
    robot.updateWeapon();

#ifdef DEBUG_LOOPTIME
    // Get looptime data
    collectLooptime();
#endif
}

#include <stdio.h>
#include <ESP32Encoder.h>
#include <algorithm> // For std::sort
#include <HardwareSerial.h>


// Robot parameters
constexpr double WHEEL_DIAMETER = 0.069;
constexpr double WHEEL_DISTANCE = 0.19;
constexpr double MAX_MOTOR_SPEED = 180.0;  // Max speed in RPS
constexpr int MAX_MOTOR_DRIVER_DUTYCYCLE = 190; //255*(3/4) because we are running 4S Lipo
constexpr int ENCODER_TICKS_PER_REVOLUTION = 14 * 2;
constexpr double ENCODER_PT1_TIMECONSTANT_S = 0.1;  //TODO
//constexpr double GEARBOX_RATIO = 1.0/200.0;
constexpr double GEARBOX_RATIO = 1.0/9.6;
constexpr int MAX_WEAPON_MOTOR_DRIVER_DUTYCYCLE = 50;

constexpr unsigned long UPDATE_INTERVAL_ENCODER = 1;  // In ms
constexpr unsigned long UPDATE_INTERVAL_PID_CONTROL = 1;  // In ms

// PID controller parameters - adjust these during tuning
double KP = 5.0;
double KI = 3.0;
double KD = 0.0;
double KI_MAX = 20.0;

// Pin definitions
constexpr int LEFT_ENCODER_C2_PIN = 19;
constexpr int LEFT_ENCODER_C1_PIN = 18;
constexpr int RIGHT_ENCODER_C1_PIN = 5;
constexpr int RIGHT_ENCODER_C2_PIN = 4;

constexpr int MOTORDRIVER_LEFT_CW_PIN = 26;
constexpr int MOTORDRIVER_LEFT_CCW_PIN = 25;
constexpr int MOTORDRIVER_LEFT_PWM_PIN = 27;

constexpr int MOTORDRIVER_RIGHT_CW_PIN = 32;
constexpr int MOTORDRIVER_RIGHT_CCW_PIN = 16;
constexpr int MOTORDRIVER_RIGHT_PWM_PIN = 17;

constexpr int MOTORDRIVER_STBY_PIN = 33;

constexpr int MOTORDRIVER_WEAPON_PIN = 23;


constexpr int HC12_RX_PIN = 22;
constexpr int HC12_TX_PIN = 21;
constexpr int HC12_SET_PIN = 15;

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
        // ledcAttach(pin3,  freq, resolution);
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
        // ledcAttach(pinPwm, freq, resolution);
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
        
        double leftWheelSpeed = linearWheelSpeed - angularWheelSpeed;
        double rightWheelSpeed = linearWheelSpeed + angularWheelSpeed;

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



class H2Transceiver {
public:
    H2Transceiver(HardwareSerial& serial, int setPin) : _serial(serial), _setPin(setPin) {}

    void init(int baud = 9600, int rx = 22, int tx = 21) {
        pinMode(_setPin, OUTPUT);
        digitalWrite(_setPin, HIGH); // Datenmodus
        _serial.begin(baud, SERIAL_8N1, rx, tx);
    }

    void update() {
        if (_serial.available() > 0) {
            String input = _serial.readStringUntil('\n');
            input.trim();

            if (input.startsWith("CMD:")) {
                // Decompositioning
                String data = input.substring(4);
                int commaIndex = data.indexOf(',');
                if (commaIndex != -1) {
                    _vel_lin = data.substring(0, commaIndex).toFloat();
                    _vel_ang = data.substring(commaIndex + 1).toFloat();
                    _newDataAvailable = true;
                }
            } 
            else if (input.startsWith("WPN:")) {
                _weapon_speed = input.substring(4).toFloat();
                _newDataAvailable = true;
            }
        }
    }

    // Getters
    float getVelLin() const { return _vel_lin; }
    float getVelAng() const { return _vel_ang; }
    float getWeaponSpeed() const { return _weapon_speed; }
    bool isNewDataAvailable() const { return _newDataAvailable; }
    
    // Reset the flag after reading
    void resetNewDataFlag() { _newDataAvailable = false; }

private:
    HardwareSerial& _serial;
    int _setPin;
    float _vel_lin = 0.0;
    float _vel_ang = 0.0;
    float _weapon_speed = 0.0;
    bool _newDataAvailable = false;
};


// Initilize HC12 Communication
HardwareSerial HC12_Serial(2);
H2Transceiver transceiver(HC12_Serial, HC12_SET_PIN);


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

void setup() {
    pinMode(MOTORDRIVER_STBY_PIN, OUTPUT);
    digitalWrite(MOTORDRIVER_STBY_PIN, HIGH);
    robot.init();
    transceiver.init(9600, HC12_RX_PIN, HC12_TX_PIN);
}


void loop() {
    transceiver.update();
    
    // Check if new data is available
    if (transceiver.isNewDataAvailable()) {
        robot.calculateAndSetWheelSpeeds(transceiver.getVelLin(), transceiver.getVelAng());
        weapon.setWeaponSpeed(transceiver.getWeaponSpeed());
        transceiver.resetNewDataFlag();
    }

    robot.updateDrives();
    robot.updateWeapon();
}

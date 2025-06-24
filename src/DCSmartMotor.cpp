#include "DCSmartMotor.h"

DCSmartMotor* DCSmartMotor::instances[MAX_MOTORS] = {nullptr};

DCSmartMotor::DCSmartMotor() {
    _encoderCount = 0;
    _targetAngle = 0;
    _stepsPerDeg = 1.0;
    _kp = _ki = _kd = 0;
    _errorSum = _lastError = 0;
    _lastTime = millis();
    _index = -1;
}

void DCSmartMotor::attach(uint8_t pwmPin, uint8_t dirPin, uint8_t encA, uint8_t encB) {
    _pwmPin = pwmPin;
    _dirPin = dirPin;
    _encA = encA;
    _encB = encB;

    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_encA, INPUT_PULLUP);
    pinMode(_encB, INPUT_PULLUP);

    for (int i = 0; i < MAX_MOTORS; i++) {
        if (instances[i] == nullptr) {
            instances[i] = this;
            _index = i;
            break;
        }
    }

    if (_index == -1) return;

    switch (_index) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(_encA), encoderA_ISR0, CHANGE);
            attachInterrupt(digitalPinToInterrupt(_encB), encoderB_ISR0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(_encA), encoderA_ISR1, CHANGE);
            attachInterrupt(digitalPinToInterrupt(_encB), encoderB_ISR1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(_encA), encoderA_ISR2, CHANGE);
            attachInterrupt(digitalPinToInterrupt(_encB), encoderB_ISR2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(_encA), encoderA_ISR3, CHANGE);
            attachInterrupt(digitalPinToInterrupt(_encB), encoderB_ISR3, CHANGE);
            break;
        case 4:
            attachInterrupt(digitalPinToInterrupt(_encA), encoderA_ISR4, CHANGE);
            attachInterrupt(digitalPinToInterrupt(_encB), encoderB_ISR4, CHANGE);
            break;
        case 5:
            attachInterrupt(digitalPinToInterrupt(_encA), encoderA_ISR5, CHANGE);
            attachInterrupt(digitalPinToInterrupt(_encB), encoderB_ISR5, CHANGE);
            break;
    }
}

void DCSmartMotor::setPID(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void DCSmartMotor::setCPR(long cpr) {
    _cpr = cpr;
}

void DCSmartMotor::setStepsPerDegree(float spd) {
    _stepsPerDeg = spd;
}

void DCSmartMotor::moveToAngle(float angle) {
    _targetAngle = angle;
}

float DCSmartMotor::currentAngle() {
    return (float)_encoderCount / _stepsPerDeg;
}

void DCSmartMotor::stop() {
    analogWrite(_pwmPin, 0);
}

void DCSmartMotor::update() {
    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0;
    if (dt <= 0) return;

    _lastTime = now;

    float current = currentAngle();
    float error = _targetAngle - current;

    _errorSum += error * dt;
    float dError = (error - _lastError) / dt;

    float output = _kp * error + _ki * _errorSum + _kd * dError;
    _lastError = error;

    bool dir = output > 0;
    digitalWrite(_dirPin, dir);
    analogWrite(_pwmPin, min(abs(output), 255.0));
}

void DCSmartMotor::encoderISR(uint8_t index, bool channelA, bool rising) {
    if (instances[index] == nullptr) return;

    DCSmartMotor* m = instances[index];
    bool A = digitalRead(m->_encA);
    bool B = digitalRead(m->_encB);

    int encoded = (A << 1) | B;
    int sum = (m->_lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        m->_encoderCount++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        m->_encoderCount--;

    m->_lastEncoded = encoded;
}

#define DEFINE_ENCODER_ISR(index) \
void encoderA_ISR##index() { \
    DCSmartMotor::encoderISR(index, true, digitalRead(DCSmartMotor::instances[index]->_encA)); \
} \
void encoderB_ISR##index() { \
    DCSmartMotor::encoderISR(index, false, digitalRead(DCSmartMotor::instances[index]->_encB)); \
}

DEFINE_ENCODER_ISR(0)
DEFINE_ENCODER_ISR(1)
DEFINE_ENCODER_ISR(2)
DEFINE_ENCODER_ISR(3)
DEFINE_ENCODER_ISR(4)
DEFINE_ENCODER_ISR(5)

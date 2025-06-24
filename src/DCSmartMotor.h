// DCSmartMotor.h
// Created by Waqar Farooq at Mechatronics Lab,
// Department of Mechanical Engineering,
// Islamic University of Science & Technology, Kashmir

#ifndef DCSMARTMOTOR_H
#define DCSMARTMOTOR_H

#include <Arduino.h>

#define MAX_MOTORS 6

class DCSmartMotor {
public:
    DCSmartMotor();

    void attach(uint8_t pwmPin, uint8_t dirPin, uint8_t encA, uint8_t encB);
    void setPID(float kp, float ki, float kd);
    void setCPR(long cpr);
    void setStepsPerDegree(float spd);
    void moveToAngle(float angle);
    float currentAngle();
    void stop();
    void update();

    static void encoderISR(uint8_t motorIndex, bool channelA, bool rising);

private:
    uint8_t _pwmPin, _dirPin, _encA, _encB;
    float _kp, _ki, _kd;
    float _targetAngle;
    float _stepsPerDeg;
    long _cpr;
    float _errorSum, _lastError;
    float _currentOutput;
    unsigned long _lastTime;

    long _encoderCount;
    int _index;
    int _lastEncoded;

    static DCSmartMotor* instances[MAX_MOTORS];

    void applyPID();

    // Grant ISR functions access to private members
    friend void encoderA_ISR0();
    friend void encoderB_ISR0();
    friend void encoderA_ISR1();
    friend void encoderB_ISR1();
    friend void encoderA_ISR2();
    friend void encoderB_ISR2();
    friend void encoderA_ISR3();
    friend void encoderB_ISR3();
    friend void encoderA_ISR4();
    friend void encoderB_ISR4();
    friend void encoderA_ISR5();
    friend void encoderB_ISR5();
};

#endif

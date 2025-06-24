#include <DCSmartMotor.h>

DCSmartMotor motor1, motor2;

void setup() {
    motor1.attach(9, 8, 2, 3);
    motor1.setPID(2.0, 0.5, 0.1);
    motor1.setCPR(1024);
    motor1.setStepsPerDegree(2.84);
    motor1.moveToAngle(90);

    motor2.attach(6, 7, 18, 19);
    motor2.setPID(1.5, 0.3, 0.1);
    motor2.setCPR(1024);
    motor2.setStepsPerDegree(2.84);
    motor2.moveToAngle(45);
}

void loop() {
    motor1.update();
    motor2.update();
}

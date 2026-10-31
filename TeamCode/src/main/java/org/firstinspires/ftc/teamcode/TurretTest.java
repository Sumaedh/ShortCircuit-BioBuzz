package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretTest {
    Servo servo;
    public double LEFT_POSITION = 0.2;
    public double CENTER_POSITION = 0.5;
    public double RIGHT_POSITION = 0.8;
    public double KP = 0.005;

    public void initTurret(HardwareMap hwMap) {
        servo = hwMap.tryGet(Servo.class, "turret");
        if (servo != null) {
            servo.setDirection(Servo.Direction.FORWARD);
            center();
        }
    }

    public boolean available() {
        return servo != null;
    }

    public void left() {
        if (servo != null) servo.setPosition(LEFT_POSITION);
    }

    public void center() {
        if (servo != null) servo.setPosition(CENTER_POSITION);
    }

    public void right() {
        if (servo != null) servo.setPosition(RIGHT_POSITION);
    }

    public void trackTargetOdo(double robotX, double robotY, double targetX, double targetY, double robotHeading) {
        if (servo == null) return;

        double xOffset = targetX - robotX;
        double yOffset = targetY - robotY;
        double angle = Math.atan2(yOffset, xOffset);
        double turretAngle = angle - robotHeading;

        while (turretAngle > Math.PI) {
            turretAngle -= 2 * Math.PI;
        }
        while (turretAngle < -Math.PI) {
            turretAngle += 2 * Math.PI;
        }

        double targetPosition = CENTER_POSITION + (turretAngle * KP);
        targetPosition = Math.max(0.0, Math.min(1.0, targetPosition));
        servo.setPosition(targetPosition);
    }
}
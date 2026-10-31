package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretServoZero {

    private final Servo turretServo;

    public TurretServoZero(HardwareMap hwMap, String servoName) {
        this.turretServo = hwMap.get(Servo.class, servoName);
    }

    public double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    public void setTurretAngleDegrees(double degrees) {
        double halfRange = TurretServoConfig.TOTAL_RANGE_DEGREES / 2.0;
        degrees = clamp(degrees, -halfRange, halfRange);

        double normalized = degrees / TurretServoConfig.TOTAL_RANGE_DEGREES;
        double targetPos = TurretServoConfig.ZERO_POSITION + normalized;

        targetPos = clamp(targetPos, TurretServoConfig.MIN_SERVO_POS, TurretServoConfig.MAX_SERVO_POS);

        turretServo.setPosition(targetPos);
    }

    public void setToZero() {
        turretServo.setPosition(TurretServoConfig.ZERO_POSITION);
    }
}
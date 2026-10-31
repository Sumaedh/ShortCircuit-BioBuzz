package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretServoCalibration {

    private final Servo turretServo;

    public TurretServoCalibration(HardwareMap hwMap, String servoName) {
        turretServo = hwMap.get(Servo.class, servoName);
    }

    public static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    public static double mapRange(double x, double inMin, double inMax, double outMin, double outMax) {
        double t = (x - inMin) / (inMax - inMin);
        return outMin + t * (outMax - outMin);
    }

    public void setServoPosition(double pos) {
        pos = clamp(pos, 0.0, 1.0);
        turretServo.setPosition(pos);
    }

    public void setToZero() {
        setServoPosition(TurretServoConfig.ZERO_POSITION);
    }

    public void setToLeftLimit() {
        setServoPosition(TurretServoConfig.LEFT_LIMIT_POS);
    }

    public void setToRightLimit() {
        setServoPosition(TurretServoConfig.RIGHT_LIMIT_POS);
    }

    public void setTargetDegrees(double degrees) {
        double mapped = mapRange(
                degrees,
                TurretServoConfig.LEFT_LIMIT_DEGREES,
                TurretServoConfig.RIGHT_LIMIT_DEGREES,
                TurretServoConfig.LEFT_LIMIT_POS,
                TurretServoConfig.RIGHT_LIMIT_POS
        );

        setServoPosition(mapped);
    }
}
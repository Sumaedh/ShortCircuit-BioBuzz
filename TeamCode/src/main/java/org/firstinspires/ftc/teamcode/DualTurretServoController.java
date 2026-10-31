package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DualTurretServoController {

    private final Servo leftServo;
    private final Servo rightServo;

    public DualTurretServoController(HardwareMap hwMap, String leftServoName, String rightServoName) {
        leftServo = hwMap.get(Servo.class, leftServoName);
        rightServo = hwMap.get(Servo.class, rightServoName);
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

    public void setToZero() {
        leftServo.setPosition(DualTurretServoConfig.ZERO_POSITION);
        rightServo.setPosition(DualTurretServoConfig.ZERO_POSITION);
    }

    public void setTargetDegrees(double degrees) {
        // Map turret angle to left/right servo positions
        double leftPos = mapRange(
                degrees,
                DualTurretServoConfig.LEFT_LIMIT_DEGREES,
                DualTurretServoConfig.RIGHT_LIMIT_DEGREES,
                DualTurretServoConfig.LEFT_LIMIT_POS,
                DualTurretServoConfig.RIGHT_LIMIT_POS
        );

        // If the two servos are mirrored, the right servo is the inverse
        double rightPos = mapRange(
                degrees,
                DualTurretServoConfig.LEFT_LIMIT_DEGREES,
                DualTurretServoConfig.RIGHT_LIMIT_DEGREES,
                DualTurretServoConfig.RIGHT_LIMIT_POS,
                DualTurretServoConfig.LEFT_LIMIT_POS
        );

        leftServo.setPosition(clamp(leftPos, 0.0, 1.0));
        rightServo.setPosition(clamp(rightPos, 0.0, 1.0));
    }
}
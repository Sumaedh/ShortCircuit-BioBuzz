package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretTest {

    private final DcMotorEx turretMotor;


    private final double ticksPerRevolution = 537.7;

    private final double gearRatio = 1.0;

    private final double minAngleDeg = -135.0;
    private final double maxAngleDeg = 135.0;

    private final double switchbackThresholdDeg = 5.0;

    private final double kP = 0.02;
    private final double kI = 0.001;
    private final double kD = 0.0008;
    private final double kF = 0.02;

    private double targetDegrees = 0.0;
    private double previousError = 0.0;
    private double integral = 0.0;
    private double previousTimestamp = -1.0;

    private double zeroOffsetDegrees = 0.0;

    public TurretTest(HardwareMap hw, String motorName) {
        this.turretMotor = hw.get(DcMotorEx.class, motorName);

        turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setZeroOffsetDegrees(double offsetDegrees) {
        this.zeroOffsetDegrees = offsetDegrees;
    }

    public static double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public static double normalizeDegrees(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    public double getCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    public double ticksToDegrees(double ticks) {
        double motorRotations = ticks / ticksPerRevolution;
        double turretRotations = motorRotations / gearRatio;
        return (turretRotations * 360.0) + zeroOffsetDegrees;
    }

    public double degreesToTicks(double degrees) {
        double turretRotations = (degrees - zeroOffsetDegrees) / 360.0;
        double motorRotations = turretRotations * gearRatio;
        return motorRotations * ticksPerRevolution;
    }

    public void setTargetDegrees(double degrees) {
        targetDegrees = clampDegrees(degrees, minAngleDeg, maxAngleDeg);
    }

    public double getTargetDegrees() {
        return targetDegrees;
    }

    public void setTargetFromField(double robotX, double robotY,
                                   double robotHeadingRadians,
                                   double targetX, double targetY) {
        double xOffset = targetX - robotX;
        double yOffset = targetY - robotY;

        double fieldAngle = Math.atan2(yOffset, xOffset);
        double relativeTargetAngle = normalizeRadians(fieldAngle - robotHeadingRadians);

        double desiredTurretDegrees = Math.toDegrees(relativeTargetAngle);

        desiredTurretDegrees = clampDegrees(desiredTurretDegrees, minAngleDeg, maxAngleDeg);

        setTargetDegrees(desiredTurretDegrees);
    }

    public double clampDegrees(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    public double clampPower(double value) {
        if (value > 1.0) return 1.0;
        if (value < -1.0) return -1.0;
        return value;
    }

    public void handleSwitchback() {
        double currentDegrees = getCurrentTurretDegrees();

        if (currentDegrees < minAngleDeg - switchbackThresholdDeg) {
            targetDegrees = minAngleDeg;
        } else if (currentDegrees > maxAngleDeg + switchbackThresholdDeg) {
            targetDegrees = maxAngleDeg;
        }
    }

    public void update() {
        double currentDegrees = getCurrentTurretDegrees();

        handleSwitchback();

        double error = normalizeDegrees(targetDegrees - currentDegrees);

        double now = System.currentTimeMillis() / 1000.0;
        double dt = 0.02;

        if (previousTimestamp > 0.0) {
            dt = now - previousTimestamp;
        }
        previousTimestamp = now;

        if (dt <= 0.0) {
            dt = 0.02;
        }

        integral += error * dt;
        double derivative = (error - previousError) / dt;

        double feedforward = kF * Math.signum(error);
        double output = kP * error + kI * integral + kD * derivative + feedforward;

        if (Math.abs(error) < 1.0) {
            integral = 0.0;
            output = 0.0;
        }

        output = clampPower(output);

        turretMotor.setPower(output);
        previousError = error;
    }

    public void stop() {
        turretMotor.setPower(0.0);
    }

    public boolean isOnTarget(double toleranceDegrees) {
        double currentDegrees = getCurrentTurretDegrees();
        double error = normalizeDegrees(targetDegrees - currentDegrees);
        return Math.abs(error) <= toleranceDegrees;
    }

    public double getCurrentTurretDegrees() {
        return ticksToDegrees(getCurrentTicks());
    }
}
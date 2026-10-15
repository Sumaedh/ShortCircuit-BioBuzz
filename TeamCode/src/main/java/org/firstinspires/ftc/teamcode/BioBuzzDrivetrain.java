package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Robot-centric mecanum drive. The signs and one-third slow mode match your old class. */
public final class BioBuzzDrivetrain {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private final boolean dryRun = BioBuzzConfig.DRY_RUN;
    public double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    public void init(HardwareMap map) {
        if (dryRun) return;
        frontLeft = map.get(DcMotor.class, BioBuzzConfig.FRONT_LEFT);
        frontRight = map.get(DcMotor.class, BioBuzzConfig.FRONT_RIGHT);
        backLeft = map.get(DcMotor.class, BioBuzzConfig.BACK_LEFT);
        backRight = map.get(DcMotor.class, BioBuzzConfig.BACK_RIGHT);
        frontLeft.setDirection(BioBuzzConfig.LEFT_DRIVE_DIRECTION);
        backLeft.setDirection(BioBuzzConfig.LEFT_DRIVE_DIRECTION);
        frontRight.setDirection(BioBuzzConfig.RIGHT_DRIVE_DIRECTION);
        backRight.setDirection(BioBuzzConfig.RIGHT_DRIVE_DIRECTION);
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void driveMecanum(double leftY, double leftX, double rightX) {
        drive(leftY, leftX, rightX, 1.0);
    }
    public void driveMecanumSlower(double leftY, double leftX, double rightX) {
        drive(leftY, leftX, rightX, 1.0 / 3.0);
    }
    private void drive(double y, double x, double turn, double scale) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1.0);
        frontLeftPower = (-y + x + turn) / denominator * scale;
        frontRightPower = (-y - x - turn) / denominator * scale;
        backLeftPower = (-y - x + turn) / denominator * scale;
        backRightPower = (-y + x - turn) / denominator * scale;
        if (!dryRun) {
            frontLeft.setPower(frontLeftPower); frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower); backRight.setPower(backRightPower);
        }
    }
    public void stop() {
        frontLeftPower = frontRightPower = backLeftPower = backRightPower = 0;
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            if (motor != null) motor.setPower(0);
        }
    }
    public Runnable driveMecanumParametric(double y, double x, double r) { driveMecanum(y, x, r); return null; }
    public Runnable driveMecanumSlowerParametric(double y, double x, double r) { driveMecanumSlower(y, x, r); return null; }
    public Runnable stopParametric() { stop(); return null; }
}
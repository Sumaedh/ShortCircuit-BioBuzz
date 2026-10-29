package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    public double NORMAL_SCALE = 1.0;
    public double SLOW_SCALE = 1.0 / 3.0;

    public void initDrivetrain(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
    }

    public void driveMecanum(double leftY, double leftX, double rightX) {
        driveFieldCentric(leftY, leftX, rightX, NORMAL_SCALE);
    }

    public void driveMecanumSlower(double leftY, double leftX, double rightX) {
        driveFieldCentric(leftY, leftX, rightX, SLOW_SCALE);
    }

    private void driveFieldCentric(double y, double x, double turn, double scale) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - (-y) * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + (-y) * Math.cos(-botHeading);

        double adjustedY = -rotY;
        double adjustedX = rotX;

        double denominator = Math.max(Math.abs(adjustedY) + Math.abs(adjustedX) + Math.abs(turn), 1.0);

        frontLeft.setPower((-adjustedY + adjustedX + turn) / denominator * scale);
        frontRight.setPower((-adjustedY - adjustedX - turn) / denominator * scale);
        backLeft.setPower((-adjustedY - adjustedX + turn) / denominator * scale);
        backRight.setPower((-adjustedY + adjustedX - turn) / denominator * scale);
    }

    public void turnToTarget(double tx) {
        double kp = 0.03;
        double error = tx;
        double turnPower = error * kp;
        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
        driveMecanum(0, 0, turnPower);
    }

    public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    public void stop() {
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            if (motor != null) {
                motor.setPower(0);
            }
        }
    }
}
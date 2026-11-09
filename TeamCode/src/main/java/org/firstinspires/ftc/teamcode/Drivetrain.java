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

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /*
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

         */
    }


    public void driveMecanum(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower((left_y + left_x + right_x) / maxPower);
        frontRight.setPower((left_y - left_x - right_x) / maxPower);
        backLeft.setPower((left_y - left_x + right_x) / maxPower);
        backRight.setPower((left_y + left_x - right_x) / maxPower);
    }

    public void driveMecanumSlower(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower(((left_y + left_x + right_x) / maxPower) / 3);
        frontRight.setPower(((left_y - left_x - right_x) / maxPower) / 3);
        backLeft.setPower(((left_y - left_x + right_x) / maxPower) / 3);
        backRight.setPower(((left_y + left_x - right_x) / maxPower) / 3);
    }


    public void driveMecanum1(double leftY, double leftX, double rightX) {
        driveFieldCentric(leftY, leftX, rightX, NORMAL_SCALE);
    }

    public void driveMecanumSlower1(double leftY, double leftX, double rightX) {
        driveFieldCentric(leftY, leftX, rightX, SLOW_SCALE);
    }

    public void driveRobotCentric(double leftY, double leftX, double rightX) {
        drive(leftY, leftX, rightX, 0.0, NORMAL_SCALE);
    }

    private void driveFieldCentric(double y, double x, double turn, double scale) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        drive(y, x, turn, botHeading, scale);
    }

    private void drive(double y, double x, double turn, double botHeading, double scale) {

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
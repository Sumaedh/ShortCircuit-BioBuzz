package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
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
    }

    public void driveMecanum(double leftY, double leftX, double rightX) {
        drive(leftY, leftX, rightX, NORMAL_SCALE);
    }

    public void driveMecanumSlower(double leftY, double leftX, double rightX) {
        drive(leftY, leftX, rightX, SLOW_SCALE);
    }

    private void drive(double y, double x, double turn, double scale) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1.0);

        frontLeft.setPower((-y + x + turn) / denominator * scale);
        frontRight.setPower((-y - x - turn) / denominator * scale);
        backLeft.setPower((-y - x + turn) / denominator * scale);
        backRight.setPower((-y + x - turn) / denominator * scale);
    }

    public void stop() {
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            if (motor != null) {
                motor.setPower(0);
            }
        }
    }
}
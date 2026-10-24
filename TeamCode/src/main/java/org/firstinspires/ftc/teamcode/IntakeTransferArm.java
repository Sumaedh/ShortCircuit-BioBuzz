package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeTransferArm {
    DcMotorEx rightIntakeMotor, leftIntakeMotor;
    Servo arm, shotStopper;
    public double INTAKE_POWER = 1.0;
    public double FEED_POWER = 1.0;
    public double ARM_UP = 0.0;
    public double ARM_INTAKE_FLOWER = 0.0;
    public double ARM_BLOCK = 0.5;
    public double SHOT_BLOCKED = 0.5;
    public double SHOT_OPEN = 0.0;

    public void initIntakeTransfer(HardwareMap hwMap) {
        rightIntakeMotor = hwMap.get(DcMotorEx.class, "rightIntakeMotor");
        leftIntakeMotor = hwMap.get(DcMotorEx.class, "leftIntakeMotor");
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor : new DcMotorEx[]{leftIntakeMotor, rightIntakeMotor}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        arm = hwMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.REVERSE);
        shotStopper = hwMap.get(Servo.class, "shotStopper");
        shotStopper.setDirection(Servo.Direction.REVERSE);
        armUp();
        stopShoot();
    }

    public void intakeOn() { setPower(INTAKE_POWER); }
    public void intakeOff() { setPower(0); }
    public void intakeReverse() { setPower(-INTAKE_POWER); }
    public void feedShooter() { setPower(FEED_POWER); }

    private void setPower(double power) {
        if (leftIntakeMotor != null) leftIntakeMotor.setPower(power);
        if (rightIntakeMotor != null) rightIntakeMotor.setPower(power);
    }
    public void armUp() { if (arm != null) arm.setPosition(ARM_UP); }
    public void intakeFlower() { if (arm != null) arm.setPosition(ARM_INTAKE_FLOWER); }
    public void armBlock() { if (arm != null) arm.setPosition(ARM_BLOCK); }
    public void stopShoot() { if (shotStopper != null) shotStopper.setPosition(SHOT_BLOCKED); }
    public void continueShoot() { if (shotStopper != null) shotStopper.setPosition(SHOT_OPEN); }
}
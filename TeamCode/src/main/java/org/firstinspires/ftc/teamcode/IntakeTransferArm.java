package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeTransferArm {
    DcMotorEx rightIntakeMotor, leftIntakeMotor;
    Servo arm, shotStopper;

    public double INTAKE_POWER = 0.8;


    public double ARM_UP = 0.0;
    public double ARM_INTAKE_FLOWER = 0.482;
    public double ARM_BLOCK = 0.25;


    //TODO: FIND THIS
    public double SHOT_BLOCKED = 0.15;
    public double SHOT_OPEN = 0.0;

    public void initIntakeTransfer(HardwareMap hwMap) {
        rightIntakeMotor = hwMap.get(DcMotorEx.class, "rightIntakeMotor");
        leftIntakeMotor = hwMap.get(DcMotorEx.class, "leftIntakeMotor");

        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hwMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.REVERSE);

        shotStopper = hwMap.get(Servo.class, "shotStopper");

        stopShoot();
        armUp();
        intakeOff();
    }

    public void intakeOn() {
        setPower(INTAKE_POWER);
    }

    public void intakeOff() {
        setPower(0);
    }

    public void intakeReverse() {
        setPower(-INTAKE_POWER);
    }

    private void setPower(double power) {
        if (leftIntakeMotor != null) leftIntakeMotor.setPower(power);
        if (rightIntakeMotor != null) rightIntakeMotor.setPower(power);
    }

    public void armUp() {
        arm.setPosition(ARM_UP);
    }

    public void intakeFlower() {
         arm.setPosition(ARM_INTAKE_FLOWER);
    }

    public void armBlock() {

        arm.setPosition(ARM_BLOCK);
    }

    public void stopShoot() {
        shotStopper.setPosition(SHOT_BLOCKED);
    }

    public void continueShoot() {
        shotStopper.setPosition(SHOT_OPEN);
    }
}
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
    public double SHOT_BLOCKED = 0;
    public double SHOT_OPEN = 0.15;

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
        leftIntakeMotor.setPower(0.8);
        rightIntakeMotor.setPower(0.25);
    }

    public void intakeOff() {
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
    }

    public void intakeReverse() {
        leftIntakeMotor.setPower(-0.5);
        rightIntakeMotor.setPower(-0.5);
    }

    public void transferShoot(){
        leftIntakeMotor.setPower(0.3);
        rightIntakeMotor.setPower(0.8);
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
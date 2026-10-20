package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeTransferArm {
    DcMotorEx rightIntakeMotor;
    DcMotorEx leftIntakeMotor;
    Servo arm;
    Servo shotStopper;

    public void initIntakeTransfer(HardwareMap hwMap) {
        rightIntakeMotor = hwMap.get(DcMotorEx.class, "rightIntakeMotor");
        leftIntakeMotor = hwMap.get(DcMotorEx.class, "leftIntakeMotor");
        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hwMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.REVERSE);

        shotStopper = hwMap.get(Servo.class, "shotStopper");
        shotStopper.setDirection(Servo.Direction.REVERSE);
    }

    public void intakeOn() {
        rightIntakeMotor.setPower(1);
        leftIntakeMotor.setPower(1);
    }

    public void stopShoot() {
        shotStopper.setPosition(0.5);
        //TODO: FIND POSITION
    }

    public void continueShoot() {
        shotStopper.setPosition(0);
    }

    //TODO: INVERSE KINEMATICS FOR ARM TO INTAKE FLOWER AND BLOCK INTAKING (3 POSITIONS)

    public void intakeOff() {
        rightIntakeMotor.setPower(0);
        leftIntakeMotor.setPower(0);
    }

    public void intakePartial() { rightIntakeMotor.setPower(0.0);}

    public void intakeReverse() {
        rightIntakeMotor.setPower(-1);
        leftIntakeMotor.setPower(-1);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {

    DcMotorEx leftShootingMotor;
    DcMotorEx rightShootingMotor;

    public double fShooting = 21.5;
    public double pShooting = 500;
    public double curTargetVelocity = 0;

    public double LONG_VELOCITY = 1470;
    public double SHORT_VELOCITY = 1200;
    public double SHOOTER_TOLERANCE = 50;

    public void initShooter(HardwareMap hwMap) {
        leftShootingMotor = hwMap.get(DcMotorEx.class, "leftShootingMotor");
        rightShootingMotor = hwMap.get(DcMotorEx.class, "rightShootingMotor");

        leftShootingMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShootingMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShootingMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, 0, fShooting)
        );

        rightShootingMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, 0, fShooting)
        );

        curTargetVelocity = 0;
        leftShootingMotor.setVelocity(0);
        rightShootingMotor.setVelocity(0);
    }

    public void setCurTargetVelocity(String distance, int customLength) {
        if ("long".equals(distance)) {
            curTargetVelocity = LONG_VELOCITY;
        } else if ("short".equals(distance)) {
            curTargetVelocity = SHORT_VELOCITY;
        } else if ("0".equals(distance) && customLength != 0) {
            curTargetVelocity = customLength;
        } else {
            curTargetVelocity = 0;
        }
    }

    public Runnable setCurTargetVelocityParametric(
            String distance, int customLength) {
        return () -> setCurTargetVelocity(distance, customLength);
    }

    public double getVelocity() {
        return leftShootingMotor.getVelocity();
    }

    public boolean shooterAtTarget() {
        return curTargetVelocity > 0
                && Math.abs(getVelocity() - curTargetVelocity)
                <= SHOOTER_TOLERANCE;
    }

    public void PIDFShootingLoop() {
        leftShootingMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, 0, fShooting)
        );

        rightShootingMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, 0, fShooting)
        );

        leftShootingMotor.setVelocity(curTargetVelocity);
        rightShootingMotor.setVelocity(curTargetVelocity);
    }

    public void stopShooter() {
        curTargetVelocity = 0;
        leftShootingMotor.setVelocity(0);
        leftShootingMotor.setPower(0);

        rightShootingMotor.setPower(0);
        rightShootingMotor.setVelocity(0);
    }
}

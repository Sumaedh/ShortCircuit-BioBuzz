package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    DcMotorEx leftShootingMotor, rightShootingMotor;

    public double fShooting = 0;
    public double pShooting = 0;
    public double iShooting = 0;
    public double dShooting = 0;


    // TODO: FIND THIS VALUE
    public double CUSTOM_TARGET_VELOCITY = 3000;
    public double curTargetVelocity = 0;
    public double SHOOTER_TOLERANCE = 50;


    private void applyShooterPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(pShooting, iShooting, dShooting, fShooting);
        leftShootingMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        rightShootingMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void initShooter(HardwareMap hwMap) {
        leftShootingMotor = hwMap.get(DcMotorEx.class, "leftShootingMotor");
        rightShootingMotor = hwMap.get(DcMotorEx.class, "rightShootingMotor");

        leftShootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        applyShooterPIDF();

        leftShootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, 0, fShooting)
        );

        curTargetVelocity = 0;
        leftShootingMotor.setVelocity(0);

        rightShootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, 0, fShooting)
        );

        curTargetVelocity = 0;
        rightShootingMotor.setVelocity(0);

    }

    public void setCurTargetVelocity(String mode) {
        if (mode.equals("on")) {
            curTargetVelocity = CUSTOM_TARGET_VELOCITY;
        } else {
            curTargetVelocity = 0;
        }

    }

    public Runnable setCurTargetVelocityParametric(
            String mode) {
        return () -> setCurTargetVelocity(mode);
    }

    public double getVelocity() {
        return leftShootingMotor.getVelocity();
    }

    public double getVelocity2() {
        return rightShootingMotor.getVelocity();
    }

    public boolean shooterAtTarget() {
        return curTargetVelocity > 0 && Math.abs(getVelocity() - curTargetVelocity) <= SHOOTER_TOLERANCE && Math.abs(getVelocity2() - curTargetVelocity) <= SHOOTER_TOLERANCE;
    }

    public void PIDFShootingLoop() {
        leftShootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, 0, fShooting)
        );

        rightShootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, 0, fShooting)
        );

        leftShootingMotor.setVelocity(curTargetVelocity);
        rightShootingMotor.setVelocity(curTargetVelocity);
    }

    public void stopShooter() {
        curTargetVelocity = 0;
        leftShootingMotor.setVelocity(0);
        leftShootingMotor.setPower(0);
        rightShootingMotor.setVelocity(0);
        rightShootingMotor.setPower(0);
    }
}

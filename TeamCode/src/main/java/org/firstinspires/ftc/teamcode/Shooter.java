package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    DcMotorEx leftShootingMotor, rightShootingMotor;

    public double fShooting = 21.5;
    public double pShooting = 500;

    public static double CUSTOM_TARGET_VELOCITY = 1470;

    public double curTargetVelocity = 0;
    public double SHORT_VELOCITY = 1200;
    public double LONG_VELOCITY = 1470;
    public double SHOOTER_TOLERANCE = 50;

    private double commandedVelocity = 0;

    public void initShooter(HardwareMap hwMap) {
        leftShootingMotor = hwMap.get(DcMotorEx.class, "leftShootingMotor");
        rightShootingMotor = hwMap.get(DcMotorEx.class, "rightShootingMotor");

        leftShootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShootingMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[]{leftShootingMotor, rightShootingMotor}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            motor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(pShooting, 0, 0, fShooting)
            );
        }

        stopShooter();
    }

    public void setCurTargetVelocity(String distance, int customLength) {
        if ("long".equals(distance)) {
            curTargetVelocity = LONG_VELOCITY;
        } else if ("short".equals(distance)) {
            curTargetVelocity = SHORT_VELOCITY;
        } else if ("custom".equals(distance)) {
            curTargetVelocity = customLength > 0 ? customLength : CUSTOM_TARGET_VELOCITY;
        } else if ("0".equals(distance) && customLength != 0) {
            curTargetVelocity = customLength;
        } else {
            curTargetVelocity = 0;
        }
    }

    public double getVelocity() {
        return leftShootingMotor.getVelocity();
    }

    public double getVelocity2() {
        return rightShootingMotor.getVelocity();
    }

    public boolean shooterAtTarget() {
        return curTargetVelocity > 0
                && Math.abs(getVelocity() - curTargetVelocity) <= SHOOTER_TOLERANCE
                && Math.abs(getVelocity2() - curTargetVelocity) <= SHOOTER_TOLERANCE;
    }

    public void PIDFShootingLoop() {
        if (curTargetVelocity <= 0) {
            if (commandedVelocity != 0) {
                coast();
            }
            return;
        }
        if (curTargetVelocity != commandedVelocity) {
            leftShootingMotor.setVelocity(curTargetVelocity);
            rightShootingMotor.setVelocity(curTargetVelocity);
            commandedVelocity = curTargetVelocity;
        }
    }

    public void stopShooter() {
        curTargetVelocity = 0;
        coast();
    }

    private void coast() {
        for (DcMotorEx motor : new DcMotorEx[]{leftShootingMotor, rightShootingMotor}) {
            if (motor != null) {
                motor.setMotorDisable();
            }
        }
        commandedVelocity = 0;
    }
}

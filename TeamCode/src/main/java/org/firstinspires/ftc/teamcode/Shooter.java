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

    public double curTargetVelocity = 0;
    public double LONG_VELOCITY = 1470;
    public double SHORT_VELOCITY = 1200;
    public double SHOOTER_TOLERANCE = 50;

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

            motor.setVelocity(0);
        }

        curTargetVelocity = 0;
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
        for (DcMotorEx motor : new DcMotorEx[]{leftShootingMotor, rightShootingMotor}) {
            motor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(pShooting, 0, 0, fShooting)
            );
            motor.setVelocity(curTargetVelocity);
        }
    }

    public void stopShooter() {
        curTargetVelocity = 0;
        for (DcMotorEx motor : new DcMotorEx[]{leftShootingMotor, rightShootingMotor}) {
            if (motor != null) {
                motor.setVelocity(0);
                motor.setPower(0);
            }
        }
    }
}
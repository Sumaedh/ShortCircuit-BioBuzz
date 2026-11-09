package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "ShooterPIDTest")
public class ShooterPIDTest extends OpMode {

    DcMotorEx leftShootingMotor, rightShootingMotor;

    public static double fShooting = 0;
    public static double pShooting = 0;
    public static double iShooting = 0;
    public static double dShooting = 0;


    // TODO: FIND THIS VALUE
    public static double CUSTOM_TARGET_VELOCITY = 000;
    public static double curTargetVelocity = 0;
    public static double SHOOTER_TOLERANCE = 50;

    private void applyShooterPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(pShooting, iShooting, dShooting, fShooting);
        leftShootingMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        rightShootingMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    @Override
    public void init() {
        leftShootingMotor = hardwareMap.get(DcMotorEx.class, "leftShootingMotor");
        rightShootingMotor = hardwareMap.get(DcMotorEx.class, "rightShootingMotor");

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

    @Override
    public void loop() {
        leftShootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, iShooting, dShooting, fShooting)
        );

        rightShootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, iShooting, dShooting, fShooting)
        );

        leftShootingMotor.setVelocity(curTargetVelocity);
        rightShootingMotor.setVelocity(curTargetVelocity);

    }
}

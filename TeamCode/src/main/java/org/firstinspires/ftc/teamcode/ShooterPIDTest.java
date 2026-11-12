package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp(name = "ShooterPIDTest")
public class ShooterPIDTest extends OpMode {

    DcMotorEx leftShootingMotor, rightShootingMotor;

    public static double fShooting = 0;
    public static double pShooting = 0;
    public static double iShooting = 0;
    public static double dShooting = 0;

    DcMotorEx rightIntakeMotor, leftIntakeMotor;
    public static double intakePower = 0;


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

        rightIntakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        leftIntakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");

        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        leftIntakeMotor.setPower(intakePower);
        rightIntakeMotor.setPower(intakePower);

        telemetry.addData("p value: ", pShooting);
        telemetry.addData("i value: ", iShooting);
        telemetry.addData("d value: ", dShooting);
        telemetry.addData("f value: ", fShooting);
        telemetry.addData("target velocity: ", curTargetVelocity);
        telemetry.addData("error (left): ", curTargetVelocity - leftShootingMotor.getVelocity());
        telemetry.addData("error (right): ", curTargetVelocity - rightShootingMotor.getVelocity());

    }
}

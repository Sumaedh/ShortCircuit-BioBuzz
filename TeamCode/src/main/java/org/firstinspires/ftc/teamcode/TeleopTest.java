package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "BIOBUZZ TeleOp Testing Hub", group = "BIOBUZZ")
public class TeleopTest extends OpMode {

    public static double pShooting = 0.0;
    public static double iShooting = 0.0;
    public static double dShooting = 0.0;
    public static double fShooting = 0.0;
    public static double curTargetVelocity = 0.0;

    public static double intakeMotorPower = 0.0;
    public static double armPos = 0.0;
    public static double pitchPos = 0.0;

    DcMotorEx leftShootingMotor;
    DcMotorEx rightShootingMotor;
    DcMotorEx rightIntakeMotor;
    DcMotorEx leftIntakeMotor;
    Servo armMotor;
    Servo pitchServo;

    @Override
    public void init() {
        leftShootingMotor = hardwareMap.get(DcMotorEx.class, "leftShootingMotor");
        rightShootingMotor = hardwareMap.get(DcMotorEx.class, "rightShootingMotor");

        leftShootingMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShootingMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyShooterPIDF();

        rightIntakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        leftIntakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");
        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hardwareMap.get(Servo.class, "arm"); // Fixed to match "arm" config string
        armMotor.setDirection(Servo.Direction.REVERSE);

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        telemetry.addData("Status", "Testing Hub Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        applyShooterPIDF();
        leftShootingMotor.setVelocity(curTargetVelocity);
        rightShootingMotor.setVelocity(curTargetVelocity);

        rightIntakeMotor.setPower(intakeMotorPower);
        leftIntakeMotor.setPower(intakeMotorPower);

        armMotor.setPosition(armPos);
        pitchServo.setPosition(pitchPos);

        sendTestingTelemetry();
    }

    private void applyShooterPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(pShooting, iShooting, dShooting, fShooting);
        leftShootingMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        rightShootingMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    private void sendTestingTelemetry() {
        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Left Velocity", leftShootingMotor.getVelocity());
        telemetry.addData("Right Velocity", rightShootingMotor.getVelocity());
        telemetry.addData("P / I / D / F", pShooting + " / " + iShooting + " / " + dShooting + " / " + fShooting);

        telemetry.addData("--- INTAKE & ARM ---", "");
        telemetry.addData("Intake Power", intakeMotorPower);
        telemetry.addData("Arm Position", armPos);

        telemetry.addData("--- PITCH SERVO ---", "");
        telemetry.addData("Pitch Position", pitchPos);
        telemetry.update();
    }

    @Override
    public void stop() {
        curTargetVelocity = 0;
        intakeMotorPower = 0;
        leftShootingMotor.setVelocity(0);
        rightShootingMotor.setVelocity(0);
        rightIntakeMotor.setPower(0);
        leftIntakeMotor.setPower(0);
    }
}
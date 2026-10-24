package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "teleop(TESTING)")
class TeleopTesting extends OpMode {

    //TODO: SHOOTER
    DcMotorEx leftShootingMotor;
    DcMotorEx rightShootingMotor;

    public static double fShooting = 0;
    public static double pShooting = 0;
    public static double dShooting = 0;
    public static double curTargetVelocity = 0;

    //TODO: INTAKE TRANSFER ARM
    DcMotorEx rightIntakeMotor;
    DcMotorEx leftIntakeMotor;
    Servo armMotor;
    public static double intakeMotorPower = 0;
    public static double armPos = 0;


    // TODO: TURRET AND PITCH
    Servo pitchServo;
    public static double pitchPos = 0;

    @Override
    public void init() {
        // TODO: SHOOTER
        leftShootingMotor = hardwareMap.get(DcMotorEx.class, "leftShootingMotor");
        rightShootingMotor = hardwareMap.get(DcMotorEx.class, "rightShootingMotor");

        leftShootingMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShootingMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShootingMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, dShooting, fShooting)
        );

        rightShootingMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, dShooting, fShooting)
        );

        curTargetVelocity = 0;
        leftShootingMotor.setVelocity(0);
        rightShootingMotor.setVelocity(0);

        // TODO: INTAKE TRANSFER ARM
        rightIntakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        leftIntakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");
        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hardwareMap.get(Servo.class, "armMotor");
        armMotor.setDirection(Servo.Direction.REVERSE);

        // TODO: TURRET AND PITCH

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
    }

    @Override
    public void loop() {

        // TODO: SHOOTER
        leftShootingMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, dShooting, fShooting)
        );

        rightShootingMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pShooting, 0, dShooting, fShooting)
        );

        leftShootingMotor.setVelocity(curTargetVelocity);
        rightShootingMotor.setVelocity(curTargetVelocity);

        //TODO: INTAKE TRANSFER ARM
        rightIntakeMotor.setPower(intakeMotorPower);
        leftIntakeMotor.setPower(intakeMotorPower);

        armMotor.setPosition(armPos);

        //TODO: TURRET AND PITCH
        pitchServo.setPosition(pitchPos);
    }
}
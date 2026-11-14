package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "TeleopMeet1TESTING")
public class TeleopMeet1Test extends OpMode {
    Drivetrain dt = new Drivetrain();

    private TelemetryManager panelsTelemetry;

    DcMotorEx rightIntakeMotor, leftIntakeMotor;
    Servo arm, shotStopper;

    public double INTAKE_POWER = 0.8;


    public double ARM_UP = 0.0;
    public double ARM_INTAKE_FLOWER = 0.46;
    public double ARM_BLOCK = 0.25;


    //TODO: FIND THIS
    public static double SHOT_BLOCKED = 0.04;
    public static double SHOT_OPEN = 0.09;


    public static double leftIntakeOn = 0.8;
    public static double rightIntakeOn = 0.2;

    public static double leftTransferOn = 1;
    public static double rightTransferOn = 1;


    //Lights lights = new Lights();

    // wfhurfrufhuxfx9hf389sijofesijoiesfjojisdopjfaopf

    Servo pitchServo;
    public static double pitchPos = 0.04;
    //0.05
    DcMotorEx leftShootingMotor, rightShootingMotor;

    public static double fShooting = 24;
    //22
    public static double pShooting = 170;
    //50
    public static double iShooting = 0;
    public static double dShooting = 0;


    // TODO: FIND THIS VALUE
    public static double CUSTOM_TARGET_VELOCITY = 965;
    //1175
    public double curTargetVelocity = 0;
    public static double SHOOTER_TOLERANCE = 50;


    private void applyShooterPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(pShooting, iShooting, dShooting, fShooting);
        leftShootingMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        rightShootingMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }



    private Follower follower;

    public void intakeOn() {
        leftIntakeMotor.setPower(leftIntakeOn);
        rightIntakeMotor.setPower(rightIntakeOn);
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
        leftIntakeMotor.setPower(leftTransferOn);
        rightIntakeMotor.setPower(rightTransferOn);
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

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        dt.initDrivetrain(hardwareMap);

        rightIntakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        leftIntakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");

        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.REVERSE);

        shotStopper = hardwareMap.get(Servo.class, "shotStopper");
        armUp();
        intakeOff();

        //lights.initLights(hardwareMap);

        //follower = Constants.create(hardwareMap);

        leftShootingMotor = hardwareMap.get(DcMotorEx.class, "leftShootingMotor");
        rightShootingMotor = hardwareMap.get(DcMotorEx.class, "rightShootingMotor");

        leftShootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShootingMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");

        curTargetVelocity = 0;
        rightShootingMotor.setVelocity(0);
    }

    @Override
    public void loop() {

        //DRIVETRAIN
        if (gamepad1.left_trigger > 0.75) {
            dt.driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            dt.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


        // PEDRO
        /*
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
        follower.update();
         */

        //SHOOTER - GAMEPAD 1 RIGHT TRIGGER



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

        pitchServo.setPosition(pitchPos);

        if (gamepad2.right_trigger > 0.75) {
            curTargetVelocity = CUSTOM_TARGET_VELOCITY;

        } else {
            curTargetVelocity = 0;
        }
        // LIGHTS
        /*
        if (shooter.shooterAtTarget() && shooter.curTargetVelocity != 0) {
            lights.ready();
        } else {
            lights.notReady();
        }

         */



        // INTAKE
        if (gamepad1.aWasPressed()) {
            intakeOn();
            stopShoot();
        } else if (gamepad1.bWasPressed()) {
            intakeOff();
            stopShoot();
        } else if (gamepad1.yWasPressed()) {
            intakeReverse();
        } else if (gamepad1.xWasPressed()) {
            transferShoot();
            continueShoot();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            stopShoot();
        } else if (gamepad1.dpadRightWasPressed()) {
            continueShoot();
        }


        // ARM
        if (gamepad1.dpadUpWasPressed()) {
            armUp();
        } else if (gamepad1.dpadDownWasPressed()) {
            intakeFlower();
        }

        double left = leftShootingMotor.getVelocity();
        double right = rightShootingMotor.getVelocity();

        panelsTelemetry.addData("curTargetVelocity", curTargetVelocity);
        panelsTelemetry.addData("left", left);
        panelsTelemetry.addData("right", right);
        panelsTelemetry.update(telemetry);
    }
}
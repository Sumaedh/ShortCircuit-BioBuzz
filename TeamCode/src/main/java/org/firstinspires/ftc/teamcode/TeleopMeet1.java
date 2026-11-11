package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.jetbrains.annotations.TestOnly;

@TeleOp(name = "TeleopMeet1")
public class TeleopMeet1 extends OpMode {
    Drivetrain dt = new Drivetrain();
    IntakeTransferArm ita = new IntakeTransferArm();
    Shooter shooter = new Shooter();
    Pitch pitch = new Pitch();

    //Lights lights = new Lights();

    // wfhurfrufhuxfx9hf389

    private Follower follower;

    @Override
    public void init() {
        dt.initDrivetrain(hardwareMap);
        ita.initIntakeTransfer(hardwareMap);
        shooter.initShooter(hardwareMap);
        pitch.initPitch(hardwareMap);
        //lights.initLights(hardwareMap);

        //follower = Constants.create(hardwareMap);
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

        //SHOOTER
        if (gamepad2.right_trigger > 0.75) {
            shooter.setCurTargetVelocity("on");
            ita.continueShoot();
        } else {
            shooter.setCurTargetVelocity("off");
            ita.stopShoot();
        }

        shooter.PIDFShootingLoop();


        // LIGHTS
        /*
        if (shooter.shooterAtTarget() && shooter.curTargetVelocity != 0) {
            lights.ready();
        } else {
            lights.notReady();
        }

         */

        // STOPPER
        if (gamepad2.left_trigger > 0.75) {
            ita.continueShoot();
        } else {
            ita.stopShoot();
        }

        // INTAKE
        if (gamepad1.aWasPressed()) {
            ita.intakeOn();
        } else if (gamepad1.bWasPressed()) {
            ita.intakeOff();
        } else if (gamepad1.yWasPressed()) {
            ita.intakeReverse();
        } else if (ita.shotStopper.getPosition() == 0) {
            ita.setPower(0.6);
        }



        // ARM
        if (gamepad1.dpadUpWasPressed()) {
            ita.armUp();
        } else if (gamepad1.dpadDownWasPressed()) {
            ita.intakeFlower();
        }

    }
}
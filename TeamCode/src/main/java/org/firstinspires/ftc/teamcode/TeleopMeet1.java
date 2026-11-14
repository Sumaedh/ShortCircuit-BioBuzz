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

        pitch.pitchForShoot();
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
        if (gamepad2.right_trigger > 0.75) {
            shooter.setCurTargetVelocity("on");
        } else {
            shooter.setCurTargetVelocity("off");
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


        // INTAKE
        if (gamepad1.aWasPressed()) {
            ita.intakeOn();
            ita.stopShoot();
        } else if (gamepad1.bWasPressed()) {
            ita.intakeOff();
            ita.stopShoot();
        } else if (gamepad1.yWasPressed()) {
            ita.intakeReverse();
        } else if (gamepad2.dpadUpWasPressed()) {
            ita.transferShoot();
            ita.continueShoot();
        }


        // ARM
        if (gamepad1.dpadUpWasPressed()) {
            ita.armUp();
        } else if (gamepad1.dpadDownWasPressed()) {
            ita.intakeFlower();
        }

        telemetry.addData("target velocity: ", shooter.curTargetVelocity);
        telemetry.addData("error (left): ", shooter.curTargetVelocity - shooter.leftShootingMotor.getVelocity());
        telemetry.addData("error (right): ", shooter.curTargetVelocity - shooter.rightShootingMotor.getVelocity());

    }
}
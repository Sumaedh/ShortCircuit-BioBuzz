package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.jetbrains.annotations.TestOnly;

@TeleOp(name = "TeleopMeet1")
public class TeleopMeet1 extends OpMode {

    Drivetrain dt = new Drivetrain();
    IntakeTransferArm ita = new IntakeTransferArm();
    Shooter shooter = new Shooter();
    Pitch pitch = new Pitch();
    // wfhurfrufhuxfx9hf389

    @Override
    public void init() {
        dt.initDrivetrain(hardwareMap);
        ita.initIntakeTransfer(hardwareMap);
        shooter.initShooter(hardwareMap);
        pitch.initPitch(hardwareMap);
    }

    @Override
    public void loop() {

        //DRIVETRAIN
        if (gamepad1.left_trigger > 0.75) {
            dt.driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            dt.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        //SHOOTER
        if (gamepad1.right_trigger > 0.75) {
            shooter.setCurTargetVelocity("on");
            ita.continueShoot();
        } else {
            shooter.setCurTargetVelocity("off");
            ita.stopShoot();
        }

        shooter.PIDFShootingLoop();

        // STOPPER
        /*
        if (shooter.shooterAtTarget() && shooter.curTargetVelocity != 0) {
            ita.continueShoot();
            ita.intakeOn();
        } else {
            ita.stopShoot();
        }

         */

        // INTAKE
        if (gamepad1.aWasPressed()) {
            ita.intakeOn();
        } else if (gamepad1.bWasPressed()) {
            ita.intakeOff();
        } else if (gamepad1.yWasPressed()) {
            ita.intakeReverse();
        }


        // ARM
        if (gamepad1.dpadUpWasPressed()) {
            ita.armUp();
        } else if (gamepad1.dpadDownWasPressed()) {
            ita.intakeFlower();
        }

    }
}
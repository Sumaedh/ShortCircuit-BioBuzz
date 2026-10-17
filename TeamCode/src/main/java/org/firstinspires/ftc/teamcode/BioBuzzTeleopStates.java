package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BIOBUZZ TeleOp States", group = "BIOBUZZ")
public class BioBuzzTeleopStates extends OpMode {
    BioBuzzDrivetrain dt = new BioBuzzDrivetrain();
    BioBuzzIntake in = new BioBuzzIntake();
    BioBuzzShooter sh = new BioBuzzShooter();
    BioBuzzShotStopper shotStopper = new BioBuzzShotStopper();
    BioBuzzIntakeStopper intakeStopper = new BioBuzzIntakeStopper();
    BioBuzzFlowerArm arm = new BioBuzzFlowerArm();
    BioBuzzBallSensors sensors = new BioBuzzBallSensors();

    enum DriveState { NORMAL, SLOW }
    enum IntakeState { OFF, ON, REVERSE, FULL, FEED }
    enum ShooterState { OFF, SHORT, LONG }
    enum StopperState { BLOCKED, OPEN }
    enum ArmState { UP, DOWN }

    DriveState driveState = DriveState.NORMAL;
    IntakeState intakeState = IntakeState.OFF;
    ShooterState shooterState = ShooterState.OFF;
    StopperState shotState = StopperState.BLOCKED;
    StopperState intakeStopperState = StopperState.BLOCKED;
    ArmState armState = ArmState.UP;
    private boolean previousA, previousY;

    @Override public void init() {
        dt.initDrivetrain(hardwareMap);
        in.initIntake(hardwareMap);
        sh.initShooter(hardwareMap);
        shotStopper.initShotStopper(hardwareMap);
        intakeStopper.initIntakeStopper(hardwareMap);
        arm.initFlowerArm(hardwareMap);
        sensors.initSensors(hardwareMap);
        telemetry.addLine("Initialized - press START");
        telemetry.update();
    }

    @Override public void loop() {
        boolean aPressed = gamepad1.a && !previousA;
        boolean yPressed = gamepad1.y && !previousY;
        previousA = gamepad1.a;
        previousY = gamepad1.y;
        boolean full = sensors.isFull();

        driveState = gamepad1.left_trigger > 0.75 ? DriveState.SLOW : DriveState.NORMAL;
        switch (driveState) {
            case NORMAL: dt.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x); break;
            case SLOW: dt.driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x); break;
        }

        if (gamepad1.back) shooterState = ShooterState.OFF;
        else if (gamepad1.right_trigger > 0.75) shooterState = ShooterState.LONG;
        else if (gamepad1.left_bumper) shooterState = ShooterState.SHORT;
        else shooterState = ShooterState.OFF;

        switch (shooterState) {
            case OFF: sh.setCurTargetVelocity("0", 0); break;
            case SHORT: sh.setCurTargetVelocity("short", 0); break;
            case LONG: sh.setCurTargetVelocity("long", 0); break;
        }
        sh.PIDFShootingLoop();

        if (gamepad1.back || gamepad1.b) intakeState = IntakeState.OFF;
        else if (gamepad1.x) intakeState = IntakeState.REVERSE;
        else if (gamepad1.dpad_up) intakeState = IntakeState.FEED;
        else if (intakeState == IntakeState.FEED) intakeState = IntakeState.OFF;
        else if (aPressed) intakeState = full ? IntakeState.FULL : IntakeState.ON;
        if (intakeState == IntakeState.ON && full) intakeState = IntakeState.FULL;

        shotState = StopperState.BLOCKED;
        intakeStopperState = StopperState.BLOCKED;
        switch (intakeState) {
            case OFF:
            case FULL:
                in.intakeOff();
                break;
            case ON:
                in.intakeOn();
                intakeStopperState = StopperState.OPEN;
                break;
            case REVERSE:
                in.intakeReverse();
                intakeStopperState = StopperState.OPEN;
                break;
            case FEED:
                if (sh.shooterAtTarget()) {
                    in.feedShooter();
                    shotState = StopperState.OPEN;
                } else {
                    in.intakeOff();
                }
                break;
        }

        switch (shotState) {
            case BLOCKED: shotStopper.block(); break;
            case OPEN: shotStopper.open(); break;
        }
        switch (intakeStopperState) {
            case BLOCKED: intakeStopper.block(); break;
            case OPEN: intakeStopper.open(); break;
        }

        if (gamepad1.back || gamepad1.dpad_down) armState = ArmState.UP;
        else if (yPressed) armState = armState == ArmState.UP ? ArmState.DOWN : ArmState.UP;
        switch (armState) {
            case UP: arm.armUp(); break;
            case DOWN: arm.armDown(); break;
        }

        telemetry.addData("Drive", driveState);
        telemetry.addData("Intake", intakeState);
        telemetry.addData("Shooter", shooterState);
        telemetry.addData("Shot shotStopper", shotState);
        telemetry.addData("Intake shotStopper", intakeStopperState);
        telemetry.addData("Flower arm", armState);
        telemetry.addData("Storage full", full);
        telemetry.addData("Shooter target", sh.curTargetVelocity);
        telemetry.addData("Shooter left", sh.getVelocity());
        telemetry.addData("Shooter right", sh.getVelocity2());
        telemetry.addData("Both shooters ready", sh.shooterAtTarget());
        telemetry.update();
    }

    @Override public void stop() {
        dt.stop();
        in.intakeOff();
        sh.stopShooter();
        shotStopper.block();
        intakeStopper.block();
        arm.armUp();
    }
}
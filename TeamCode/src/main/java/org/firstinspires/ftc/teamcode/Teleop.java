package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class Teleop extends OpMode {
    private final BioBuzzDrivetrain drivetrain = new BioBuzzDrivetrain();
    private final BioBuzzIntake intake = new BioBuzzIntake();
    private final BioBuzzShooter shooter = new BioBuzzShooter();
    private final BioBuzzShotStopper shotStopper = new BioBuzzShotStopper();
    private final BioBuzzFlowerArm arm = new BioBuzzFlowerArm();
    private final BioBuzzBallSensors sensors = new BioBuzzBallSensors();

    private enum DriveState { NORMAL, SLOW }
    private enum IntakeState { OFF, ON, REVERSE, FULL, FEED }
    private enum ShooterState { OFF, SHORT, LONG }
    private enum ArmState { UP, DOWN, BLOCK }

    private DriveState driveState = DriveState.NORMAL;
    private IntakeState intakeState = IntakeState.OFF;
    private ShooterState shooterState = ShooterState.OFF;
    private ArmState armState = ArmState.UP;

    private boolean previousA = false;
    private boolean previousY = false;

    @Override
    public void init() {
        drivetrain.initDrivetrain(hardwareMap);
        intake.initIntake(hardwareMap);
        shooter.initShooter(hardwareMap);
        shotStopper.initShotStopper(hardwareMap);
        arm.initFlowerArm(hardwareMap);

        sensors.FULL_REQUIRES_BOTH = false;
        sensors.initSensors(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        sensors.update();

        boolean aPressed = gamepad1.a && !previousA;
        boolean yPressed = gamepad1.y && !previousY;
        previousA = gamepad1.a;
        previousY = gamepad1.y;

        boolean full = sensors.isFull();

        updateDriveState();
        updateShooterState();
        updateIntakeState(aPressed, full);
        updateArmState(yPressed, full);

        applyDriveState();
        applyShooterState();
        applyIntakeState(full);
        applyArmState();

        telemetry.addData("Drive State", driveState);
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Flower Arm State", armState);
        telemetry.addData("Storage Full", full);
        sensors.addTelemetry(telemetry);
        telemetry.update();
    }

    private void updateDriveState() {
        driveState = (gamepad1.left_trigger > 0.75) ? DriveState.SLOW : DriveState.NORMAL;
    }

    private void updateShooterState() {
        switch (shooterState) {
            case OFF:
                shooterState = (gamepad1.right_trigger > 0.75) ? ShooterState.LONG :
                        (gamepad1.left_bumper ? ShooterState.SHORT : ShooterState.OFF);
                break;
            case SHORT:
                shooterState = (gamepad1.back || !gamepad1.left_bumper) ? ShooterState.OFF :
                        (gamepad1.right_trigger > 0.75 ? ShooterState.LONG : ShooterState.SHORT);
                break;
            case LONG:
                shooterState = (gamepad1.back || gamepad1.right_trigger <= 0.75) ? ShooterState.OFF :
                        (gamepad1.left_bumper ? ShooterState.SHORT : ShooterState.LONG);
                break;
        }
    }

    private void updateIntakeState(boolean aPressed, boolean full) {
        switch (intakeState) {
            case OFF:
                intakeState = gamepad1.x ? IntakeState.REVERSE :
                        gamepad1.dpad_up ? IntakeState.FEED :
                        aPressed ? (full ? IntakeState.FULL : IntakeState.ON) : IntakeState.OFF;
                break;
            case ON:
                intakeState = (gamepad1.back || gamepad1.b) ? IntakeState.OFF :
                        gamepad1.x ? IntakeState.REVERSE :
                        gamepad1.dpad_up ? IntakeState.FEED :
                        full ? IntakeState.FULL : IntakeState.ON;
                break;
            case REVERSE:
                intakeState = (gamepad1.back || gamepad1.b || !gamepad1.x) ? IntakeState.OFF : IntakeState.REVERSE;
                break;
            case FULL:
                intakeState = (gamepad1.back || gamepad1.b) ? IntakeState.OFF :
                        gamepad1.x ? IntakeState.REVERSE :
                        gamepad1.dpad_up ? IntakeState.FEED : IntakeState.FULL;
                break;
            case FEED:
                intakeState = (gamepad1.back || gamepad1.b || !gamepad1.dpad_up) ? IntakeState.OFF : IntakeState.FEED;
                break;
        }
    }

    private void updateArmState(boolean yPressed, boolean full) {
        switch (armState) {
            case UP:
                armState = full ? ArmState.BLOCK :
                        gamepad1.dpad_left ? ArmState.BLOCK :
                        yPressed ? ArmState.DOWN : ArmState.UP;
                break;
            case DOWN:
                armState = (gamepad1.back || gamepad1.dpad_down) ? ArmState.UP :
                        gamepad1.dpad_left ? ArmState.BLOCK :
                        yPressed ? ArmState.UP : ArmState.DOWN;
                break;
            case BLOCK:
                armState = (gamepad1.back || gamepad1.dpad_down) ? ArmState.UP :
                        yPressed ? ArmState.DOWN : ArmState.BLOCK;
                break;
        }
    }

    private void applyDriveState() {
        if (driveState == DriveState.NORMAL) {
            drivetrain.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drivetrain.driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    private void applyShooterState() {
        switch (shooterState) {
            case OFF:
                shooter.setCurTargetVelocity("0", 0);
                break;
            case SHORT:
                shooter.setCurTargetVelocity("short", 0);
                break;
            case LONG:
                shooter.setCurTargetVelocity("long", 0);
                break;
        }
        shooter.PIDFShootingLoop();
    }

    private void applyIntakeState(boolean full) {
        switch (intakeState) {
            case OFF:
            case FULL:
                intake.intakeOff();
                break;
            case ON:
                intake.intakeOn();
                break;
            case REVERSE:
                intake.intakeReverse();
                break;
            case FEED:
                if (shooter.shooterAtTarget()) {
                    intake.setPower(intake.FEED_POWER);
                } else {
                    intake.setPower(0);
                }
                break;
        }
    }

    private void applyArmState() {
        switch (armState) {
            case UP:
                arm.armUp();
                break;
            case DOWN:
                arm.armDown();
                break;
            case BLOCK:
                arm.armBlock();
                break;
        }
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.intakeOff();
        shooter.stopShooter();
        shotStopper.block();
        arm.armUp();
        sensors.close();
    }
}
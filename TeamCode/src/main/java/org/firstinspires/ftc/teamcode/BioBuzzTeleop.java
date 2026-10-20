package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BIOBUZZ TeleOp States", group = "BIOBUZZ")
public class BioBuzzTeleop extends OpMode {
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
    enum ArmState { UP, DOWN, BLOCK }

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

        updateDriveState();
        updateShooterState();
        updateIntakeState(aPressed, full);
        updateArmState(yPressed);

        executeDriveStateMachine();
        executeShooterStateMachine();
        executeIntakeAndStopperStateMachine();
        executeArmStateMachine();

        sendTelemetry(full);
    }


    private void updateDriveState() {
        switch (driveState) {
            case NORMAL:
                if (gamepad1.left_trigger > 0.75) {
                    driveState = DriveState.SLOW;
                }
                break;
            case SLOW:
                if (gamepad1.left_trigger <= 0.75) {
                    driveState = DriveState.NORMAL;
                }
                break;
        }
    }

    private void updateShooterState() {
        switch (shooterState) {
            case OFF:
                if (gamepad1.right_trigger > 0.75) {
                    shooterState = ShooterState.LONG;
                } else if (gamepad1.left_bumper) {
                    shooterState = ShooterState.SHORT;
                }
                break;
            case SHORT:
                if (gamepad1.back || !gamepad1.left_bumper) {
                    shooterState = ShooterState.OFF;
                } else if (gamepad1.right_trigger > 0.75) {
                    shooterState = ShooterState.LONG;
                }
                break;
            case LONG:
                if (gamepad1.back || gamepad1.right_trigger <= 0.75) {
                    shooterState = ShooterState.OFF;
                } else if (gamepad1.left_bumper) {
                    shooterState = ShooterState.SHORT;
                }
                break;
        }
    }

    private void updateIntakeState(boolean aPressed, boolean full) {
        switch (intakeState) {
            case OFF:
                if (gamepad1.x) {
                    intakeState = IntakeState.REVERSE;
                } else if (gamepad1.dpad_up) {
                    intakeState = IntakeState.FEED;
                } else if (aPressed) {
                    intakeState = full ? IntakeState.FULL : IntakeState.ON;
                }
                break;
            case ON:
                if (gamepad1.back || gamepad1.b) {
                    intakeState = IntakeState.OFF;
                } else if (gamepad1.x) {
                    intakeState = IntakeState.REVERSE;
                } else if (gamepad1.dpad_up) {
                    intakeState = IntakeState.FEED;
                } else if (full) {
                    intakeState = IntakeState.FULL;
                }
                break;
            case REVERSE:
                if (gamepad1.back || gamepad1.b || !gamepad1.x) {
                    intakeState = IntakeState.OFF;
                }
                break;
            case FULL:
                if (gamepad1.back || gamepad1.b) {
                    intakeState = IntakeState.OFF;
                } else if (gamepad1.x) {
                    intakeState = IntakeState.REVERSE;
                } else if (gamepad1.dpad_up) {
                    intakeState = IntakeState.FEED;
                }
                break;
            case FEED:
                if (gamepad1.back || gamepad1.b || !gamepad1.dpad_up) {
                    intakeState = IntakeState.OFF;
                }
                break;
        }
    }

    private void updateArmState(boolean yPressed) {
        switch (armState) {
            case UP:
                if (gamepad1.dpad_left) {
                    armState = ArmState.BLOCK;
                } else if (yPressed) {
                    armState = ArmState.DOWN;
                }
                break;
            case DOWN:
                if (gamepad1.back || gamepad1.dpad_down) {
                    armState = ArmState.UP;
                } else if (gamepad1.dpad_left) {
                    armState = ArmState.BLOCK;
                } else if (yPressed) {
                    armState = ArmState.UP;
                }
                break;
            case BLOCK:
                if (gamepad1.back || gamepad1.dpad_down) {
                    armState = ArmState.UP;
                } else if (yPressed) {
                    armState = ArmState.DOWN;
                }
                break;
        }
    }


    private void executeDriveStateMachine() {
        switch (driveState) {
            case NORMAL:
                dt.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                break;
            case SLOW:
                dt.driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                break;
        }
    }

    private void executeShooterStateMachine() {
        switch (shooterState) {
            case OFF:
                sh.setCurTargetVelocity("0", 0);
                break;
            case SHORT:
                sh.setCurTargetVelocity("short", 0);
                break;
            case LONG:
                sh.setCurTargetVelocity("long", 0);
                break;
        }
        sh.PIDFShootingLoop();
    }

    private void executeIntakeAndStopperStateMachine() {
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
    }

    private void executeArmStateMachine() {
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

    private void sendTelemetry(boolean full) {
        telemetry.addData("Drive State", driveState);
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shot Stopper", shotState);
        telemetry.addData("Intake Stopper", intakeStopperState);
        telemetry.addData("Flower Arm State", armState);
        telemetry.addData("Storage Full", full);
        telemetry.addData("Shooter Target Vel", sh.curTargetVelocity);
        telemetry.addData("Shooter Left Vel", sh.getVelocity());
        telemetry.addData("Shooter Right Vel", sh.getVelocity2());
        telemetry.addData("Shooters Ready", sh.shooterAtTarget());
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
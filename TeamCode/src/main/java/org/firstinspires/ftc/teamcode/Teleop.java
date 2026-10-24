package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Teleop extends OpMode {
    private enum ShooterState { OFF, READY, FEEDING }
    private enum IntakeState { OFF, REVERSE, INTAKING, FULL }
    private enum ArmState { UP, INTAKE_FLOWER, BLOCK }
    private enum ShotDistance { SHORT, LONG }

    private final Drivetrain drivetrain = new Drivetrain();
    private final IntakeTransferArm mechanisms = new IntakeTransferArm();
    private final Shooter shooter = new Shooter();
    private final Sensors sensors = new Sensors();

    private IntakeState intakeState = IntakeState.OFF;
    private ArmState armState = ArmState.UP;
    private ShooterState shooterState = ShooterState.OFF;
    private ShotDistance shotDistance = ShotDistance.SHORT;

    private boolean intakeEnabled = false;

    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;
    private boolean previousA = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousB = false;
    private boolean previousDpadUp = false;

    @Override
    public void init() {
        drivetrain.initDrivetrain(hardwareMap);
        mechanisms.initIntakeTransfer(hardwareMap);
        shooter.initShooter(hardwareMap);
        sensors.initSensors(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive
        drivetrain.driveMecanum(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // Intake FSM
        if (gamepad2.right_bumper && !previousRightBumper) {
            intakeEnabled = !intakeEnabled;
        }
        previousRightBumper = gamepad2.right_bumper;

        if (gamepad2.left_bumper) {
            intakeState = IntakeState.REVERSE;
        } else if (!intakeEnabled) {
            intakeState = IntakeState.OFF;
        } else if (sensors.isFull()) {
            intakeState = IntakeState.FULL;
        } else {
            intakeState = IntakeState.INTAKING;
        }

        // Arm FSM
        if (gamepad2.x && !previousX) {
            armState = ArmState.INTAKE_FLOWER;
        } else if (gamepad2.y && !previousY) {
            armState = ArmState.BLOCK;
        } else if (gamepad2.b && !previousB) {
            armState = ArmState.UP;
        }
        previousX = gamepad2.x;
        previousY = gamepad2.y;
        previousB = gamepad2.b;

        // Shot distance FSM
        if (gamepad2.dpad_up && !previousDpadUp) {
            shotDistance = (shotDistance == ShotDistance.SHORT)
                    ? ShotDistance.LONG
                    : ShotDistance.SHORT;

            if (shooterState != ShooterState.OFF) {
                if (shotDistance == ShotDistance.LONG) {
                    shooter.setCurTargetVelocity("long", 0);
                } else {
                    shooter.setCurTargetVelocity("short", 0);
                }
            }
        }
        previousDpadUp = gamepad2.dpad_up;

        // Shooter FSM
        if (gamepad2.a && !previousA) {
            if (shooterState == ShooterState.OFF) {
                shooterState = ShooterState.READY;
            } else {
                shooterState = ShooterState.OFF;
            }

            if (shooterState == ShooterState.READY) {
                if (shotDistance == ShotDistance.LONG) {
                    shooter.setCurTargetVelocity("long", 0);
                } else {
                    shooter.setCurTargetVelocity("short", 0);
                }
            }
        }
        previousA = gamepad2.a;

        if (gamepad2.right_trigger > 0.25 && shooterState == ShooterState.READY) {
            shooterState = ShooterState.FEEDING;
        } else if (gamepad2.right_trigger <= 0.25 && shooterState == ShooterState.FEEDING) {
            shooterState = ShooterState.READY;
        }

        switch (intakeState) {
            case OFF:
                mechanisms.intakeOff();
                break;
            case REVERSE:
                mechanisms.intakeReverse();
                break;
            case FULL:
                mechanisms.intakeOff();
                break;
            case INTAKING:
                mechanisms.intakeOn();
                break;
        }

        switch (armState) {
            case UP:
                mechanisms.armUp();
                break;
            case INTAKE_FLOWER:
                mechanisms.intakeFlower();
                break;
            case BLOCK:
                mechanisms.armBlock();
                break;
        }

        switch (shooterState) {
            case OFF:
                shooter.stopShooter();
                mechanisms.stopShoot();
                break;
            case READY:
                if (shotDistance == ShotDistance.LONG) {
                    shooter.setCurTargetVelocity("long", 0);
                } else {
                    shooter.setCurTargetVelocity("short", 0);
                }
                mechanisms.stopShoot();
                break;
            case FEEDING:
                if (shotDistance == ShotDistance.LONG) {
                    shooter.setCurTargetVelocity("long", 0);
                } else {
                    shooter.setCurTargetVelocity("short", 0);
                }
                mechanisms.continueShoot();
                mechanisms.feedShooter();
                break;
        }

        // Telemetry
        telemetry.addData("Intake", intakeState);
        telemetry.addData("Arm", armState);
        telemetry.addData("Shooter", shooterState);
        telemetry.addData("Distance", shotDistance);
        telemetry.addData("Intake enabled", intakeEnabled);
        telemetry.addData("Storage full", sensors.isFull());
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class Teleop extends OpMode {
    private enum ShooterState { OFF, READY, FEEDING }
    private enum IntakeState { OFF, REVERSE, INTAKING, FULL }
    private enum ArmState { UP, INTAKE_FLOWER, BLOCK }

    private final Drivetrain drivetrain = new Drivetrain();
    private final IntakeTransferArm mechanisms = new IntakeTransferArm();
    private final Shooter shooter = new Shooter();
    private final Sensors sensors = new Sensors();

    private IntakeState intakeState = IntakeState.OFF;
    private ArmState armState = ArmState.UP;
    private ShooterState shooterState = ShooterState.OFF;

    private boolean intakeEnabled = false;

    private boolean previousRightBumper = false;
    private boolean previousA = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousB = false;

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
        drivetrain.driveMecanum(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        if (gamepad2.right_bumper && !previousRightBumper) {
            intakeEnabled = !intakeEnabled;
        }
        previousRightBumper = gamepad2.right_bumper;

        boolean aPressed = gamepad2.a && !previousA;
        boolean xPressed = gamepad2.x && !previousX;
        boolean yPressed = gamepad2.y && !previousY;
        boolean bPressed = gamepad2.b && !previousB;

        previousA = gamepad2.a;
        previousX = gamepad2.x;
        previousY = gamepad2.y;
        previousB = gamepad2.b;

        switch (intakeState) {
            case OFF:
                mechanisms.intakeOff();
                if (gamepad2.left_bumper) {
                    intakeState = IntakeState.REVERSE;
                } else if (intakeEnabled) {
                    intakeState = sensors.isFull() ? IntakeState.FULL : IntakeState.INTAKING;
                }
                break;
            case REVERSE:
                mechanisms.intakeReverse();
                if (!gamepad2.left_bumper) {
                    intakeState = IntakeState.OFF;
                }
                break;
            case FULL:
                mechanisms.intakeOff();
                if (gamepad2.left_bumper) {
                    intakeState = IntakeState.REVERSE;
                } else if (!intakeEnabled) {
                    intakeState = IntakeState.OFF;
                } else if (!sensors.isFull()) {
                    intakeState = IntakeState.INTAKING;
                }
                break;
            case INTAKING:
                mechanisms.intakeOn();
                if (gamepad2.left_bumper) {
                    intakeState = IntakeState.REVERSE;
                } else if (!intakeEnabled) {
                    intakeState = IntakeState.OFF;
                } else if (sensors.isFull()) {
                    intakeState = IntakeState.FULL;
                }
                break;
        }

        switch (armState) {
            case UP:
                mechanisms.armUp();
                if (xPressed) {
                    armState = ArmState.INTAKE_FLOWER;
                } else if (yPressed) {
                    armState = ArmState.BLOCK;
                }
                break;
            case INTAKE_FLOWER:
                mechanisms.intakeFlower();
                if (bPressed) {
                    armState = ArmState.UP;
                } else if (yPressed) {
                    armState = ArmState.BLOCK;
                }
                break;
            case BLOCK:
                mechanisms.armBlock();
                if (bPressed) {
                    armState = ArmState.UP;
                } else if (xPressed) {
                    armState = ArmState.INTAKE_FLOWER;
                }
                break;
        }

        switch (shooterState) {
            case OFF:
                shooter.stopShooter();
                mechanisms.stopShoot();
                if (aPressed) {
                    shooterState = ShooterState.READY;
                }
                break;
            case READY:
                shooter.setCurTargetVelocity("custom", (int) Shooter.CUSTOM_TARGET_VELOCITY);
                shooter.PIDFShootingLoop();
                mechanisms.stopShoot();
                if (aPressed) {
                    shooterState = ShooterState.OFF;
                } else if (gamepad2.right_trigger > 0.25) {
                    shooterState = ShooterState.FEEDING;
                }
                break;
            case FEEDING:
                shooter.setCurTargetVelocity("custom", (int) Shooter.CUSTOM_TARGET_VELOCITY);
                shooter.PIDFShootingLoop();
                mechanisms.continueShoot();
                if (aPressed) {
                    shooterState = ShooterState.OFF;
                } else if (gamepad2.right_trigger <= 0.25) {
                    shooterState = ShooterState.READY;
                }
                break;
        }

        telemetry.addData("Intake", intakeState);
        telemetry.addData("Arm", armState);
        telemetry.addData("Shooter", shooterState);
        telemetry.addData("Intake enabled", intakeEnabled);
        telemetry.addData("Storage full", sensors.isFull());
        telemetry.update();
    }

    @Override
    public void stop() {
        sensors.close();
    }
}
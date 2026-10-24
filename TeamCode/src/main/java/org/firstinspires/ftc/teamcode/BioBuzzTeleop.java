package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BIOBUZZ TeleOp States", group = "BIOBUZZ")
public class BioBuzzTeleop extends OpMode {
    BioBuzzDrivetrain dt = new BioBuzzDrivetrain();
    BioBuzzIntake in = new BioBuzzIntake();
    BioBuzzShooter sh = new BioBuzzShooter();
    BioBuzzShotStopper shotStopper = new BioBuzzShotStopper();
    BioBuzzFlowerArm arm = new BioBuzzFlowerArm();
    Sensors sensors = new Sensors();

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
        arm.initFlowerArm(hardwareMap);

        sensors.STORAGE_MODE = Sensors.StorageMode.DISTANCE_CM;
        sensors.initSensors(hardwareMap);

        telemetry.addLine("Initialized - press START");
        telemetry.update();
    }

    @Override public void loop() {
        sensors.update();

        boolean aPressed = gamepad1.a && !previousA;
        boolean yPressed = gamepad1.y && !previousY;
        previousA = gamepad1.a;
        previousY = gamepad1.y;
        boolean full = sensors.isFull();

        // 2. FSM State Transitions (incorporating distance sensor 'full' check into arm state)
        updateDriveState();
        updateShooterState();
        updateIntakeState(aPressed, full);
        updateArmState(yPressed, full);

        // 3. State Execution Machines
        executeDriveStateMachine();
        executeShooterStateMachine();
        executeIntakeAndStopperStateMachine(full);
        executeArmStateMachine();

        sendTelemetry(full);
    }

    private void updateDriveState() {
        driveState = (gamepad1.left_trigger > 0.75) ? DriveState.SLOW : DriveState.NORMAL;
    }

    private void updateShooterState() {
        switch (shooterState) {
            case OFF:
                shooterState = evaluateOffShooterState();
                break;
            case SHORT:
                shooterState = evaluateShortShooterState();
                break;
            case LONG:
                shooterState = evaluateLongShooterState();
                break;
        }
    }

    private ShooterState evaluateOffShooterState() {
        return (gamepad1.right_trigger > 0.75) ? ShooterState.LONG :
                (gamepad1.left_bumper) ? ShooterState.SHORT : ShooterState.OFF;
    }

    private ShooterState evaluateShortShooterState() {
        return (gamepad1.back || !gamepad1.left_bumper) ? ShooterState.OFF :
                (gamepad1.right_trigger > 0.75) ? ShooterState.LONG : ShooterState.SHORT;
    }

    private ShooterState evaluateLongShooterState() {
        return (gamepad1.back || gamepad1.right_trigger <= 0.75) ? ShooterState.OFF :
                (gamepad1.left_bumper) ? ShooterState.SHORT : ShooterState.LONG;
    }

    private void updateIntakeState(boolean aPressed, boolean full) {
        switch (intakeState) {
            case OFF:
                intakeState = evaluateOffIntakeState(aPressed, full);
                break;
            case ON:
                intakeState = evaluateOnIntakeState(full);
                break;
            case REVERSE:
                intakeState = evaluateReverseIntakeState();
                break;
            case FULL:
                intakeState = evaluateFullIntakeState();
                break;
            case FEED:
                intakeState = evaluateFeedIntakeState();
                break;
        }
    }

    private IntakeState evaluateOffIntakeState(boolean aPressed, boolean full) {
        return gamepad1.x ? IntakeState.REVERSE :
                gamepad1.dpad_up ? IntakeState.FEED :
                aPressed ? (full ? IntakeState.FULL : IntakeState.ON) : IntakeState.OFF;
    }

    private IntakeState evaluateOnIntakeState(boolean full) {
        return (gamepad1.back || gamepad1.b) ? IntakeState.OFF :
                gamepad1.x ? IntakeState.REVERSE :
                gamepad1.dpad_up ? IntakeState.FEED :
                full ? IntakeState.FULL : IntakeState.ON;
    }

    private IntakeState evaluateReverseIntakeState() {
        return (gamepad1.back || gamepad1.b || !gamepad1.x) ? IntakeState.OFF : IntakeState.REVERSE;
    }

    private IntakeState evaluateFullIntakeState() {
        return (gamepad1.back || gamepad1.b) ? IntakeState.OFF :
                gamepad1.x ? IntakeState.REVERSE :
                gamepad1.dpad_up ? IntakeState.FEED : IntakeState.FULL;
    }

    private IntakeState evaluateFeedIntakeState() {
        return (gamepad1.back || gamepad1.b || !gamepad1.dpad_up) ? IntakeState.OFF : IntakeState.FEED;
    }

    private void updateArmState(boolean yPressed, boolean full) {
        switch (armState) {
            case UP:
                // Automatically transition to BLOCK if distance sensors detect full storage, or use manual dpad_left / y
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
            case OFF: sh.setCurTargetVelocity("0", 0); break;
            case SHORT: sh.setCurTargetVelocity("short", 0); break;
            case LONG: sh.setCurTargetVelocity("long", 0); break;
        }
        sh.PIDFShootingLoop();
    }

    private void executeIntakeAndStopperStateMachine(boolean full) {
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
                executeFeedActions();
                break;
        }

        executeStopperOutputs();
    }

    private void executeFeedActions() {
        boolean ready = sh.shooterAtTarget();
        in.setPower(ready ? in.FEED_POWER : 0.0);
        shotState = ready ? StopperState.OPEN : StopperState.BLOCKED;
    }

    private void executeStopperOutputs() {
        switch (shotState) {
            case BLOCKED: shotStopper.block(); break;
            case OPEN: shotStopper.open(); break;
        }
    }

    private void executeArmStateMachine() {
        switch (armState) {
            case UP: arm.armUp(); break;
            case DOWN: arm.armDown(); break;
            case BLOCK: arm.armBlock(); break;
        }
    }

    private void sendTelemetry(boolean full) {
        telemetry.addData("Drive State", driveState);
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shot Stopper", shotState);
        telemetry.addData("Intake Stopper", intakeStopperState);
        telemetry.addData("Flower Arm State", armState);
        telemetry.addData("Storage Full (Distance)", full);
        sensors.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override public void stop() {
        dt.stop();
        in.intakeOff();
        sh.stopShooter();
        shotStopper.block();
        arm.armUp();
        sensors.close();
    }
}
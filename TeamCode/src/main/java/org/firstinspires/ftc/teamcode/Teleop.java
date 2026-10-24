package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BIOBUZZ TeleOp States", group = "BIOBUZZ")
public class Teleop extends OpMode {
    protected final Drivetrain dt = new Drivetrain();
    protected final IntakeTransferArm ita = new IntakeTransferArm();
    protected final Turret turret = new Turret();
    protected final Shooter sh = new Shooter();
    protected final Sensors sensors = new Sensors();

    enum DriveState { NORMAL, SLOW }
    enum IntakeState { OFF, ON, REVERSE, FULL, WAIT_FOR_SPEED, FEED, SENSOR_FAULT }
    enum IntakeEvent { NONE, STOP, COLLECT, REVERSE, FEED }
    enum ShooterState { OFF, SHORT, LONG }
    enum StopperState { BLOCKED, OPEN }
    enum ArmState { UP, INTAKEFLOWER, BLOCK }
    enum TurretState { CENTER, LEFT, RIGHT }

    DriveState driveState = DriveState.NORMAL;
    IntakeState intakeState = IntakeState.OFF;
    ShooterState shooterState = ShooterState.OFF;
    StopperState shotState = StopperState.BLOCKED;
    StopperState intakeStopperState = StopperState.BLOCKED;
    ArmState armState = ArmState.UP;
    TurretState turretState = TurretState.CENTER;
    protected double triggerThreshold = 0.75;
    protected boolean enableLimelight = true;
    protected boolean enableLogitech = true;
    private boolean previousA, previousY, previousRight;

    protected void applyTuning() { }

    @Override public void init() {
        applyTuning();
        dt.initDrivetrain(hardwareMap);
        ita.initIntakeTransfer(hardwareMap);
        sh.initShooter(hardwareMap);
        turret.initTurret(hardwareMap);
        sensors.initStorage(hardwareMap);
        if (enableLimelight) sensors.initLimelight(hardwareMap);
        if (enableLogitech) sensors.initLogitech(hardwareMap);
        telemetry.addLine("Initialized - press START");
        telemetry.update();
    }

    @Override public void init_loop() {
        sensors.update();
        sensors.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override public void loop() {
        applyTuning();
        boolean aPressed = gamepad1.a && !previousA;
        boolean yPressed = gamepad1.y && !previousY;
        boolean rightPressed = gamepad1.dpad_right && !previousRight;
        previousRight = gamepad1.dpad_right;
        previousA = gamepad1.a;
        previousY = gamepad1.y;
        sensors.update();

        switch (driveState) {
            case NORMAL:
                if (gamepad1.left_trigger > triggerThreshold) driveState = DriveState.SLOW;
                break;
            case SLOW:
                if (gamepad1.left_trigger <= triggerThreshold) driveState = DriveState.NORMAL;
                break;
        }
        switch (driveState) {
            case NORMAL:
                dt.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                break;
            case SLOW:
                dt.driveMecanumSlower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                break;
        }
        if (gamepad1.back) shooterState = ShooterState.OFF;
        else {
            switch (shooterState) {
                case OFF:
                    if (gamepad1.right_trigger > triggerThreshold) shooterState = ShooterState.LONG;
                    else if (gamepad1.left_bumper) shooterState = ShooterState.SHORT;
                    break;
                case SHORT:
                    if (gamepad1.right_trigger > triggerThreshold) shooterState = ShooterState.LONG;
                    else if (!gamepad1.left_bumper) shooterState = ShooterState.OFF;
                    break;
                case LONG:
                    if (gamepad1.right_trigger <= triggerThreshold) {
                        shooterState = gamepad1.left_bumper ? ShooterState.SHORT : ShooterState.OFF;
                    }
                    break;
            }
        }
        switch (shooterState) {
            case OFF: sh.setCurTargetVelocity("0", 0); break;
            case SHORT: sh.setCurTargetVelocity("short", 0); break;
            case LONG: sh.setCurTargetVelocity("long", 0); break;
        }
        sh.PIDFShootingLoop();

        switch (requestedIntake(aPressed)) {
            case STOP: intakeState = IntakeState.OFF; break;
            case REVERSE: intakeState = IntakeState.REVERSE; break;
            case COLLECT: intakeState = IntakeState.ON; break;
            case FEED:
                if (intakeState != IntakeState.FEED) intakeState = IntakeState.WAIT_FOR_SPEED;
                break;
            case NONE: break;
        }
        switch (intakeState) {
            case ON:
                if (!sensors.storageReady()) intakeState = IntakeState.SENSOR_FAULT;
                else if (sensors.isFull()) intakeState = IntakeState.FULL;
                break;
            case WAIT_FOR_SPEED:
                if (!gamepad1.dpad_up) intakeState = IntakeState.OFF;
                else if (sh.shooterAtTarget()) intakeState = IntakeState.FEED;
                break;
            case FEED:
                if (!gamepad1.dpad_up) intakeState = IntakeState.OFF;
                else if (!sh.shooterAtTarget()) intakeState = IntakeState.WAIT_FOR_SPEED;
                break;
            case REVERSE:
                if (!gamepad1.x) intakeState = IntakeState.OFF;
                break;
            case OFF:
            case FULL:
            case SENSOR_FAULT:
                break;
        }

        shotState = StopperState.BLOCKED;
        intakeStopperState = StopperState.BLOCKED;
        switch (intakeState) {
            case ON:
                ita.intakeOn();
                intakeStopperState = StopperState.OPEN;
                break;
            case REVERSE:
                ita.intakeReverse();
                intakeStopperState = StopperState.OPEN;
                break;
            case FEED:
                ita.feedShooter();
                shotState = StopperState.OPEN;
                break;
            case OFF:
            case FULL:
            case WAIT_FOR_SPEED:
            case SENSOR_FAULT:
                ita.intakeOff();
                break;
        }
        switch (shotState) {
            case BLOCKED: ita.stopShoot(); break;
            case OPEN: ita.continueShoot(); break;
        }

        boolean blockArm = !sensors.storageReady() || sensors.isFull()
                || intakeState == IntakeState.FULL || intakeState == IntakeState.SENSOR_FAULT;
        if (gamepad1.back || gamepad1.dpad_down) armState = ArmState.UP;
        else if (gamepad1.dpad_left) armState = ArmState.BLOCK;
        else {
            switch (armState) {
                case UP:
                    if (yPressed) armState = blockArm ? ArmState.BLOCK : ArmState.INTAKEFLOWER;
                    break;
                case INTAKEFLOWER:
                    if (yPressed) armState = ArmState.UP;
                    else if (blockArm) armState = ArmState.BLOCK;
                    break;
                case BLOCK:
                    if (yPressed && !blockArm) armState = ArmState.INTAKEFLOWER;
                    break;
            }
        }
        switch (armState) {
            case UP: ita.armUp(); break;
            case INTAKEFLOWER: ita.intakeFlower(); break;
            case BLOCK: ita.armBlock(); break;
        }

        if (gamepad1.back || gamepad1.right_bumper) turretState = TurretState.CENTER;
        else if (rightPressed) {
            switch (turretState) {
                case CENTER: turretState = TurretState.LEFT; break;
                case LEFT: turretState = TurretState.RIGHT; break;
                case RIGHT: turretState = TurretState.CENTER; break;
            }
        }
        switch (turretState) {
            case CENTER: turret.center(); break;
            case LEFT: turret.left(); break;
            case RIGHT: turret.right(); break;
        }
        telemetry.addData("Turret present / state", turret.available() + " / " + turretState);

        telemetry.addData("Drive", driveState);
        telemetry.addData("Intake", intakeState);
        telemetry.addData("Shooter", shooterState);
        telemetry.addData("Shot / intake stopper", shotState + " / " + intakeStopperState);
        telemetry.addData("Flower arm", armState);
        telemetry.addData("Shooter target", sh.curTargetVelocity);
        telemetry.addData("Shooter left / right", sh.getVelocity() + " / " + sh.getVelocity2());
        telemetry.addData("Both shooters ready", sh.shooterAtTarget());
        sensors.addTelemetry(telemetry);
        telemetry.update();
    }

    private IntakeEvent requestedIntake(boolean aPressed) {
        if (gamepad1.back || gamepad1.b) return IntakeEvent.STOP;
        if (gamepad1.x) return IntakeEvent.REVERSE;
        if (gamepad1.dpad_up) return IntakeEvent.FEED;
        if (aPressed) return IntakeEvent.COLLECT;
        return IntakeEvent.NONE;
    }

    @Override public void stop() {
        dt.stop();
        ita.intakeOff();
        sh.stopShooter();
        ita.stopShoot();ita.armUp();
        turret.center();
        sensors.close();
    }
}
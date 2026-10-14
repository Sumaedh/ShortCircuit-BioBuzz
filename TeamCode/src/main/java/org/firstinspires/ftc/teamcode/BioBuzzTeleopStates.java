package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** One-gamepad TeleOp with separate finite-state machines for each mechanism. */
@TeleOp(name = "BIOBUZZ TeleOp States", group = "BIOBUZZ")
public class BioBuzzTeleopStates extends OpMode {
    private final BioBuzzDrivetrain drive = new BioBuzzDrivetrain();
    private final BioBuzzMechanisms robot = new BioBuzzMechanisms();
    private final States states = new States();
    private final Input input = new Input();

    @Override public void init() {
        BioBuzzConfig.validateLiveHardware();
        try { drive.init(hardwareMap); robot.init(hardwareMap); }
        catch (RuntimeException ex) { drive.stop(); robot.stopAll(); throw ex; }
        telemetry.addLine(BioBuzzConfig.DRY_RUN ? "DRY RUN: no hardware is mapped or moved" : "Initialized");
        telemetry.addLine("A intake | B off | X reverse | Y arm | D-pad up feed");
    }
    @Override public void loop() {
        input.a = gamepad1.a; input.b = gamepad1.b; input.x = gamepad1.x; input.y = gamepad1.y;
        input.feed = gamepad1.dpad_up; input.stow = gamepad1.dpad_down;
        input.close = gamepad1.left_bumper; input.far = gamepad1.right_trigger > 0.75;
        input.slow = gamepad1.left_trigger > 0.75; input.stop = gamepad1.back;
        input.leftY = gamepad1.left_stick_y; input.leftX = gamepad1.left_stick_x;
        input.rightX = gamepad1.right_stick_x;
        if (BioBuzzConfig.DRY_RUN) {
            robot.sensors.setSimulatedDetection(gamepad1.left_stick_button, gamepad1.right_stick_button);
        }
        states.update(input, robot, drive, BioBuzzConfig.now());
        telemetry.addData("Drive state", states.driveState);
        telemetry.addData("Intake state", states.intakeState);
        telemetry.addData("Shooter state", states.shooterState);
        telemetry.addData("Shot stopper state", states.shotGateState);
        telemetry.addData("Intake stopper state", states.intakeGateState);
        telemetry.addData("Flower arm state", states.armState);
        robot.telemetry(telemetry);
        telemetry.update();
    }
    @Override public void stop() { drive.stop(); robot.stopAll(); }

    // A small input snapshot also lets the FSM be tested without a Control Hub.
    public static final class Input {
        public boolean a, b, x, y, feed, stow, close, far, slow, stop;
        public double leftY, leftX, rightX;
    }
    public enum DriveState { NORMAL, SLOW }
    public enum IntakeState { OFF, ON, REVERSE, FULL }
    public enum ShooterState { OFF, CLOSE, FAR }
    public enum GateState { BLOCKED, OPEN }
    public enum ArmState { UP, DOWN }

    public static final class States {
        public DriveState driveState = DriveState.NORMAL;
        public IntakeState intakeState = IntakeState.OFF;
        public ShooterState shooterState = ShooterState.OFF;
        public GateState shotGateState = GateState.BLOCKED;
        public GateState intakeGateState = GateState.BLOCKED;
        public ArmState armState = ArmState.UP;
        private boolean previousA, previousY;

        public void update(Input in, BioBuzzMechanisms robot, BioBuzzDrivetrain drive, double now) {
            boolean aPressed = in.a && !previousA;
            boolean yPressed = in.y && !previousY;
            previousA = in.a; previousY = in.y;
            robot.sensors.update(now);

            // Driving remains available while any mechanism state is active.
            driveState = in.slow ? DriveState.SLOW : DriveState.NORMAL;
            switch (driveState) {
                case SLOW: drive.driveMecanumSlower(in.leftY, in.leftX, in.rightX); break;
                case NORMAL: drive.driveMecanum(in.leftY, in.leftX, in.rightX); break;
            }

            // Both shooter selectors together means OFF, matching your earlier controls.
            shooterState = in.stop || in.close == in.far ? ShooterState.OFF
                    : in.close ? ShooterState.CLOSE : ShooterState.FAR;
            switch (shooterState) {
                case CLOSE: robot.shooter.shooterClose(); break;
                case FAR: robot.shooter.shooterFar(); break;
                case OFF: robot.shooter.shooterOff(); break;
            }
            robot.shooter.update(now);

            // Stop wins over reverse; reverse wins over intake-on.
            if (in.stop || in.b) intakeState = IntakeState.OFF;
            else if (in.x) intakeState = IntakeState.REVERSE;
            else if (aPressed) intakeState = robot.sensors.isFull() ? IntakeState.FULL : IntakeState.ON;
            if (intakeState == IntakeState.ON && robot.sensors.isFull()) intakeState = IntakeState.FULL;
            // FULL stays latched until A is pressed again after clearing, or B/X is used.

            // Y toggles arm position once per press; D-pad down explicitly stows it.
            if (in.stop || in.stow) armState = ArmState.UP;
            else if (yPressed) armState = armState == ArmState.UP ? ArmState.DOWN : ArmState.UP;
            switch (armState) {
                case UP: robot.flowerArm.armUp(); break;
                case DOWN: robot.flowerArm.armDown(); break;
            }

            // A held feed request first stops collection, then waits for BOTH flywheels.
            // B, X or Back cancel feeding. No ball-count assumption is made here.
            boolean feedRequested = in.feed && !in.stop && !in.b && !in.x;
            if (feedRequested) intakeState = IntakeState.OFF;
            shotGateState = feedRequested && robot.shooter.isReady() ? GateState.OPEN : GateState.BLOCKED;
            intakeGateState = !feedRequested && (intakeState == IntakeState.ON || intakeState == IntakeState.REVERSE)
                    ? GateState.OPEN : GateState.BLOCKED;

            switch (intakeState) {
                case ON: robot.intake.intakeOn(); break;
                case REVERSE: robot.intake.intakeReverse(); break;
                case FULL:
                case OFF: robot.intake.intakeOff(); break;
            }
            switch (intakeGateState) {
                case OPEN: robot.intakeStopper.open(); break;
                case BLOCKED: robot.intakeStopper.block(); break;
            }
            switch (shotGateState) {
                case OPEN: robot.shotStopper.open(); robot.intake.feedShooter(); break;
                case BLOCKED: robot.shotStopper.block(); break;
            }
        }
    }
}
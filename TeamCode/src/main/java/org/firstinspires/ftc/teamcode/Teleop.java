package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
@TeleOp(name = "BIO-BUZZ TeleOp", group = "BIO-BUZZ")
public class Teleop extends OpMode {
    private static final double TRIGGER_THRESHOLD     = 0.25;
    private static final double SPIN_UP_TIMEOUT_SECONDS = 3.0;
    private static final double FULL_DEBOUNCE_SECONDS = 0.15;
    private static final boolean INVERT_FORWARD       = false;

    private enum IntakeState  { OFF, INTAKING, FULL, REVERSE }
    private enum ArmState     { UP, INTAKE_FLOWER, BLOCK }
    private enum ShooterState { OFF, SPINNING_UP, READY, FEEDING }

    private IntakeState  intakeState  = IntakeState.OFF;
    private ArmState     armState     = ArmState.UP;
    private ShooterState shooterState = ShooterState.OFF;

    private final Drivetrain        drivetrain = new Drivetrain();
    private final IntakeTransferArm mechanisms = new IntakeTransferArm();
    private final Shooter           shooter    = new Shooter();
    private final Sensors           sensors    = new Sensors();

    private final Button toggleIntakeBtn = new Button();
    private final Button shooterBtn      = new Button();
    private final Button armFlowerBtn    = new Button();
    private final Button armBlockBtn     = new Button();
    private final Button armUpBtn        = new Button();
    private final Button resetYawBtn     = new Button();

    private boolean intakeEnabled = false;
    private boolean reverseHeld   = false;
    private boolean feedHeld      = false;
    private boolean storageFull   = false;
    private boolean speedCheckBypassed = false;

    private final ElapsedTime shooterStateTimer = new ElapsedTime();
    private final ElapsedTime fullDebounceTimer = new ElapsedTime();
    private boolean lastRawFull = false;

    @Override
    public void init() {
        BulkCaching.enableAuto(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);
        mechanisms.initIntakeTransfer(hardwareMap);
        shooter.initShooter(hardwareMap);
        sensors.initSensors(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        shooterStateTimer.reset();
        fullDebounceTimer.reset();
    }

    @Override
    public void loop() {
        readInputs();
        updateStates();
        applyOutputs();
        updateTelemetry();
    }

    @Override
    public void stop() {
        shooter.stopShooter();
        mechanisms.intakeOff();
        mechanisms.stopShoot();
        sensors.close();
    }

    private void readInputs() {
        toggleIntakeBtn.update(gamepad2.right_bumper);
        shooterBtn.update(gamepad2.a);
        armFlowerBtn.update(gamepad2.x);
        armBlockBtn.update(gamepad2.y);
        armUpBtn.update(gamepad2.b);
        resetYawBtn.update(gamepad1.back);

        if (resetYawBtn.pressed()) {
            drivetrain.resetYaw();
        }

        if (toggleIntakeBtn.pressed()) {
            intakeEnabled = !intakeEnabled;
        }
        reverseHeld = gamepad2.left_bumper;
        feedHeld    = gamepad2.right_trigger > TRIGGER_THRESHOLD;

        updateStorageFull();
    }

    private void updateStorageFull() {
        if (!intakeEnabled) {
            return;
        }
        boolean rawFull = sensors.isFull();
        if (rawFull != lastRawFull) {
            lastRawFull = rawFull;
            fullDebounceTimer.reset();
        }
        if (fullDebounceTimer.seconds() >= FULL_DEBOUNCE_SECONDS) {
            storageFull = rawFull;
        }
    }

    private void updateStates() {
        intakeState = nextIntakeState(intakeState);
        armState    = nextArmState(armState);

        ShooterState next = nextShooterState(shooterState);
        if (next != shooterState) {
            shooterState = next;
            shooterStateTimer.reset();
        }
    }

    private IntakeState nextIntakeState(IntakeState current) {
        if (reverseHeld) {
            return IntakeState.REVERSE;
        }
        if (!intakeEnabled) {
            return IntakeState.OFF;
        }
        return storageFull ? IntakeState.FULL : IntakeState.INTAKING;
    }

    private ArmState nextArmState(ArmState current) {
        switch (current) {
            case UP:
                if (armFlowerBtn.pressed()) return ArmState.INTAKE_FLOWER;
                if (armBlockBtn.pressed())  return ArmState.BLOCK;
                return ArmState.UP;
            case INTAKE_FLOWER:
                if (armUpBtn.pressed())     return ArmState.UP;
                if (armBlockBtn.pressed())  return ArmState.BLOCK;
                return ArmState.INTAKE_FLOWER;
            case BLOCK:
                if (armUpBtn.pressed())     return ArmState.UP;
                if (armFlowerBtn.pressed()) return ArmState.INTAKE_FLOWER;
                return ArmState.BLOCK;
            default:
                return ArmState.UP;
        }
    }

    private ShooterState nextShooterState(ShooterState current) {
        switch (current) {
            case OFF:
                speedCheckBypassed = false;
                return shooterBtn.pressed() ? ShooterState.SPINNING_UP : ShooterState.OFF;
            case SPINNING_UP:
                if (shooterBtn.pressed()) return ShooterState.OFF;
                if (shooter.shooterAtTarget()) return ShooterState.READY;
                if (shooterStateTimer.seconds() >= SPIN_UP_TIMEOUT_SECONDS) {
                    speedCheckBypassed = true;
                    return ShooterState.READY;
                }
                return ShooterState.SPINNING_UP;
            case READY:
                if (shooterBtn.pressed()) return ShooterState.OFF;
                return feedHeld && (speedCheckBypassed || shooter.shooterAtTarget())
                        ? ShooterState.FEEDING : ShooterState.READY;
            case FEEDING:
                if (shooterBtn.pressed()) return ShooterState.OFF;
                return feedHeld ? ShooterState.FEEDING : ShooterState.READY;
            default:
                return ShooterState.OFF;
        }
    }

    private void applyOutputs() {
        applyDrive();
        applyIntake();
        applyArm();
        applyShooter();
    }

    private void applyDrive() {
        double forward = INVERT_FORWARD ? -gamepad1.left_stick_y : gamepad1.left_stick_y;
        drivetrain.driveRobotCentric(forward, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    private void applyIntake() {
        if (shooterState == ShooterState.FEEDING && !reverseHeld) {
            mechanisms.feedShooter();
            return;
        }
        switch (intakeState) {
            case INTAKING: mechanisms.intakeOn();      break;
            case REVERSE:  mechanisms.intakeReverse(); break;
            case FULL:
            case OFF:
            default:       mechanisms.intakeOff();     break;
        }
    }

    private void applyArm() {
        switch (armState) {
            case INTAKE_FLOWER: mechanisms.intakeFlower(); break;
            case BLOCK:         mechanisms.armBlock();     break;
            case UP:
            default:            mechanisms.armUp();        break;
        }
    }

    private void applyShooter() {
        switch (shooterState) {
            case SPINNING_UP:
            case READY:
                runFlywheel();
                mechanisms.stopShoot();
                break;
            case FEEDING:
                runFlywheel();
                mechanisms.continueShoot();
                break;
            case OFF:
            default:
                shooter.stopShooter();
                mechanisms.stopShoot();
                break;
        }
    }

    private void runFlywheel() {
        shooter.setCurTargetVelocity("custom", (int) Shooter.CUSTOM_TARGET_VELOCITY);
        shooter.PIDFShootingLoop();
    }

    private void updateTelemetry() {
        telemetry.addData("Intake", intakeState);
        telemetry.addData("Intake enabled", intakeEnabled);
        telemetry.addData("Storage full", storageFull);
        telemetry.addData("Arm", armState);
        telemetry.addData("Shooter", shooterState);
        telemetry.addData("Shooter state time", "%.2f s", shooterStateTimer.seconds());
        telemetry.addData("Shooter target", shooter.curTargetVelocity);
        telemetry.addData("Shooter left / right", "%.0f / %.0f", shooter.getVelocity(), shooter.getVelocity2());
        if (speedCheckBypassed) {
            telemetry.addLine("WARNING: flywheel never reached speed, check battery and encoders");
        }
        telemetry.update();
    }

    private static class Button {
        private boolean last = false;
        private boolean pressed = false;

        void update(boolean current) {
            pressed = current && !last;
            last = current;
        }

        boolean pressed() {
            return pressed;
        }
    }
}

 */
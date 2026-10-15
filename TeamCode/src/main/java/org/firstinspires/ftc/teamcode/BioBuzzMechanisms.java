package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shared mechanisms used by TeleOp and the four autonomous routines. */
public final class BioBuzzMechanisms {
    public final BioBuzzIntake intake = new BioBuzzIntake();
    public final BioBuzzShooter shooter = new BioBuzzShooter();
    public final BioBuzzShotStopper shotStopper = new BioBuzzShotStopper();
    public final BioBuzzIntakeStopper intakeStopper = new BioBuzzIntakeStopper();
    public final BioBuzzFlowerArm flowerArm = new BioBuzzFlowerArm();
    public final BioBuzzBallSensors sensors = new BioBuzzBallSensors();
    private boolean collecting, feeding, fullLatched;

    public void init(HardwareMap map) {
        BioBuzzConfig.validateLiveHardware();
        intake.init(map); shooter.init(map);
        shotStopper.init(map); intakeStopper.init(map); flowerArm.init(map);
        sensors.init(map);
    }
    public void update(double now) {
        sensors.update(now);
        shooter.update(now);
        if (collecting && sensors.isFull()) fullLatched = true;
        if (collecting) {
            shotStopper.block();
            if (fullLatched) { intake.intakeOff(); intakeStopper.block(); }
            else { intakeStopper.open(); intake.intakeOn(); }
        } else if (feeding) {
            intakeStopper.block();
            if (shooter.isReady()) { shotStopper.open(); intake.feedShooter(); }
            else { shotStopper.block(); intake.intakeOff(); }
        }
    }
    public void beginCollect() {
        collecting = true; feeding = false; fullLatched = sensors.isFull();
        shotStopper.block();
    }
    public void endCollect() {
        collecting = false; intake.intakeOff(); intakeStopper.block();
    }
    public void beginFeed() { collecting = false; feeding = true; intakeStopper.block(); }
    public void endFeed() { feeding = false; shotStopper.block(); intake.intakeOff(); }
    public boolean collectionFull() { return fullLatched; }
    public void stopAll() {
        collecting = false; feeding = false;
        intake.intakeOff(); shooter.shooterOff();
        shotStopper.block(); intakeStopper.block(); flowerArm.armUp();
    }
    public void telemetry(Telemetry t) {
        t.addData("Hardware output", BioBuzzConfig.DRY_RUN ? "SIMULATED ONLY" : "LIVE");
        t.addData("Full sensors 1 / 2", sensors.firstDetected() + " / " + sensors.secondDetected());
        t.addData("Full-storage detection", sensors.isFull());
        t.addData("Intake power command", intake.getPower());
        t.addData("Shooter target ticks/s", shooter.getTargetVelocity());
        t.addData("Shooter measured ticks/s", shooter.getLeftVelocity() + " / " + shooter.getRightVelocity());
        t.addData("Both shooters ready", shooter.isReady());
    }
    public Runnable beginCollectParametric() { beginCollect(); return null; }
    public Runnable endCollectParametric() { endCollect(); return null; }
    public Runnable beginFeedParametric() { beginFeed(); return null; }
    public Runnable endFeedParametric() { endFeed(); return null; }
    public Runnable stopAllParametric() { stopAll(); return null; }
}
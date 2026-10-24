package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class BioBuzzAutoActions {
    private enum CollectionState { OFF, COLLECTING, FULL }
    private enum ShotState { SPIN_UP, FEEDING, COMPLETE, FAILED }
    private enum DriveState { FOLLOWING, COMPLETE, FAILED }
    private final Follower follower;
    private final IntakeTransferArm ita;
    private final Shooter shooter;
    private final BioBuzzIntakeStopper intakeStopper;
    private final Sensors sensors;
    private CollectionState collectionState = CollectionState.OFF;
    private boolean collectingFlower, failed;
    private String status = "Initialized";

    public double ARM_SETTLE_SECONDS = 0.40;
    public double PICKUP_DWELL_SECONDS = 0.70;
    public double FEED_SECONDS = 1.50;
    public double SHOT_TIMEOUT_SECONDS = 5.0;
    public double PATH_TIMEOUT_SECONDS = 10.0;

    public BioBuzzAutoActions(Follower follower, IntakeTransferArm ita, Shooter shooter,
                              BioBuzzIntakeStopper intakeStopper, Sensors sensors) {
        this.follower = follower;
        this.ita = ita;
        this.shooter = shooter;
        this.intakeStopper = intakeStopper; // Fixed: assigned the parameter correctly
        this.sensors = sensors;
    }

    private static double now() { return System.nanoTime() * 1e-9; } // Fixed: added multiplication operator
    public String status() { return status; }
    public boolean failed() { return failed; }

    public static Interpolator linearHeading(Pose from, Pose to) {
        double start = from.heading();
        double change = Angle.error(start, to.heading());
        return (curve, t) -> {
            double progress = curve.length() > 0 ? 1.0 - (curve.remainingDistance(t) / curve.length()) : t; // Fixed: added subtraction and division operators
            return Angle.normalize(start + (change * Math.max(0.0, Math.min(1.0, progress)))); // Fixed: added multiplication operator
        };
    }

    private Command pause(double seconds) {
        final double[] started = {0};
        return Command.build().setStart(() -> started[0] = now())
                .setDone(() -> (now() - started[0]) >= seconds); // Fixed: added subtraction operator
    }

    public Command preparePickup(boolean flower) {
        return sequential(
                instant(() -> { status = "Prepare pickup"; stopMechanisms(); }),
                instant(() -> { if (flower) ita.intakeFlower(); else ita.armUp(); }),
                pause(ARM_SETTLE_SECONDS),
                instant(() -> {
                    status = "Collecting";
                    collectingFlower = flower;
                    collectionState = CollectionState.COLLECTING;
                    updateCollection();
                }));
    }

    public void updateCollection() {
        switch (collectionState) {
            case OFF: return;
            case COLLECTING:
                if (!sensors.storageReady()) {
                    fail("Storage sensor unavailable");
                    return;
                }
                if (sensors.isFull()) collectionState = CollectionState.FULL;
                break;
            case FULL: break;
        }
        switch (collectionState) {
            case COLLECTING:
                intakeStopper.open();
                ita.intakeOn();
                break;
            case FULL:
                ita.intakeOff();
                intakeStopper.block();
                if (collectingFlower) ita.armBlock();
                break;
            case OFF: break;
        }
    }

    public Command finishPickup() {
        final double[] started = {0};
        return sequential(
                Command.build().setStart(() -> started[0] = now())
                        .setDone(() -> !failed && (collectionState == CollectionState.FULL
                                || (now() - started[0]) >= PICKUP_DWELL_SECONDS)), // Fixed: added subtraction operator
                instant(() -> { status = "Stow after pickup"; stopMechanisms(); }),
                pause(ARM_SETTLE_SECONDS));
    }

    public Command shoot(boolean far, String label) {
        final double[] started = {0}, last = {0}, fed = {0};
        final ShotState[] state = {ShotState.SPIN_UP};
        return Command.build().requiring(shooter, ita)
                .setStart(() -> {
                    status = label;
                    stopMechanisms();
                    shooter.setCurTargetVelocity(far ? "long" : "short", 0);
                    started[0] = last[0] = now();
                    fed[0] = 0;
                    state[0] = ShotState.SPIN_UP;
                })
                .setExecute(() -> {
                    double time = now();
                    if (state[0] == ShotState.FEEDING) fed[0] += time - last[0];
                    last[0] = time;
                    switch (state[0]) {
                        case SPIN_UP:
                            if (shooter.shooterAtTarget()) state[0] = ShotState.FEEDING;
                            break;
                        case FEEDING:
                            if (fed[0] >= FEED_SECONDS) state[0] = ShotState.COMPLETE;
                            else if (!shooter.shooterAtTarget()) state[0] = ShotState.SPIN_UP;
                            break;
                        case COMPLETE:
                        case FAILED: break;
                    }
                    if (time - started[0] > SHOT_TIMEOUT_SECONDS && state[0] != ShotState.COMPLETE) {
                        state[0] = ShotState.FAILED;
                        fail(label + " timed out");
                    }
                    switch (state[0]) {
                        case FEEDING: ita.continueShoot(); ita.feedShooter(); break;
                        case SPIN_UP: ita.stopShoot(); ita.intakeOff(); break;
                        case COMPLETE:
                        case FAILED: stopMechanisms(); break;
                    }
                })
                .setDone(() -> !failed && state[0] == ShotState.COMPLETE)
                .setEnd(reason -> stopMechanisms());
    }

    public Command drive(Path path, String label) {
        final double[] started = {0};
        final DriveState[] state = {DriveState.FOLLOWING};
        return Command.build().requiring(follower)
                .setStart(() -> {
                    status = label; started[0] = now();
                    state[0] = DriveState.FOLLOWING; follower.follow(path);
                })
                .setDone(() -> {
                    switch (state[0]) {
                        case FOLLOWING:
                            if (failed) state[0] = DriveState.FAILED;
                            else if (!follower.following()) state[0] = DriveState.COMPLETE;
                            else if ((now() - started[0]) > PATH_TIMEOUT_SECONDS) { // Fixed: added subtraction operator
                                state[0] = DriveState.FAILED;
                                fail(label + " timed out");
                            }
                            break;
                        case COMPLETE:
                        case FAILED: break;
                    }
                    return state[0] == DriveState.COMPLETE && !failed;
                });
    }

    private void fail(String message) {
        failed = true;
        status = message;
        stopMechanisms();
    }

    public void stopMechanisms() {
        collectionState = CollectionState.OFF;
        ita.intakeOff(); shooter.stopShooter();
        ita.stopShoot(); intakeStopper.block(); ita.armUp();
    }
}
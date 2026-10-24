package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class AutoActions {
    private enum CollectionState { OFF, COLLECTING, FULL }
    private enum ShotState { SPIN_UP, FEEDING, COMPLETE, FAILED }
    private enum DriveState { FOLLOWING, COMPLETE, FAILED }

    private final Follower follower;
    private final IntakeTransferArm intake;
    private final Shooter shooter;
    private final Sensors sensors;

    private CollectionState collectionState = CollectionState.OFF;
    private boolean collectingFlower, failed;
    private String status = "Initialized";

    public double ARM_SETTLE_SECONDS = 0.40;
    public double PICKUP_DWELL_SECONDS = 0.70;
    public double FEED_SECONDS = 1.50;
    public double SHOT_TIMEOUT_SECONDS = 5.0;
    public double PATH_TIMEOUT_SECONDS = 10.0;

    public AutoActions(
            Follower follower,
            IntakeTransferArm intake,
            Shooter shooter,
            Sensors sensors) {
        this.follower = follower;
        this.intake = intake;
        this.shooter = shooter;
        this.sensors = sensors;
    }

    private static double now() {
        return System.nanoTime() * 1e-9;
    }

    public String status() {
        return status;
    }

    public boolean failed() {
        return failed;
    }

    public static Interpolator linearHeading(Pose from, Pose to) {
        double start = from.heading();
        double change = Angle.error(start, to.heading());
        return (curve, t) -> {
            double progress = curve.length() > 0
                    ? 1.0 - (curve.remainingDistance(t) / curve.length())
                    : t;
            progress = Math.max(0.0, Math.min(1.0, progress));
            return Angle.normalize(start + change * progress);
        };
    }

    private Command pause(double seconds) {
        final double[] started = {0};
        return Command.build()
                .setStart(() -> started[0] = now())
                .setDone(() -> (now() - started[0]) >= seconds);
    }

    public Command preparePickup(boolean flower) {
        return sequential(
                instant(() -> {
                    status = "Prepare pickup";
                    stopMechanisms();
                }),
                instant(() -> {
                    if (flower) intake.armBlock();
                    else intake.armUp();
                }),
                pause(ARM_SETTLE_SECONDS),
                instant(() -> {
                    status = "Collecting";
                    collectingFlower = flower;
                    collectionState = CollectionState.COLLECTING;
                    updateCollection();
                })
        );
    }

    public void updateCollection() {
        switch (collectionState) {
            case OFF:
                return;
            case COLLECTING:
                if (sensors.isFull()) collectionState = CollectionState.FULL;
                break;
            case FULL:
                break;
        }

        switch (collectionState) {
            case COLLECTING:
                intake.intakeOn();
                break;
            case FULL:
                intake.intakeOff();
                if (collectingFlower) intake.armBlock();
                break;
            case OFF:
                break;
        }
    }

    public Command finishPickup() {
        final double[] started = {0};
        return sequential(
                Command.build()
                        .setStart(() -> started[0] = now())
                        .setDone(() -> !failed
                                && (collectionState == CollectionState.FULL
                                || (now() - started[0]) >= PICKUP_DWELL_SECONDS)),
                instant(() -> {
                    status = "Stow after pickup";
                    stopMechanisms();
                }),
                pause(ARM_SETTLE_SECONDS)
        );
    }

    public Command shoot(boolean far, String label) {
        final double[] started = {0}, last = {0}, fed = {0};
        final ShotState[] state = {ShotState.SPIN_UP};

        return Command.build().requiring(shooter)
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
                        case FAILED:
                            break;
                    }

                    if (time - started[0] > SHOT_TIMEOUT_SECONDS && state[0] != ShotState.COMPLETE) {
                        state[0] = ShotState.FAILED;
                        fail(label + " timed out");
                    }

                    switch (state[0]) {
                        case FEEDING:
                            intake.feedShooter();
                            intake.continueShoot();
                            break;
                        case SPIN_UP:
                            intake.intakeOff();
                            intake.stopShoot();
                            break;
                        case COMPLETE:
                        case FAILED:
                            stopMechanisms();
                            break;
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
                    status = label;
                    started[0] = now();
                    state[0] = DriveState.FOLLOWING;
                    follower.follow(path);
                })
                .setDone(() -> {
                    switch (state[0]) {
                        case FOLLOWING:
                            if (failed) state[0] = DriveState.FAILED;
                            else if (!follower.following()) state[0] = DriveState.COMPLETE;
                            else if ((now() - started[0]) > PATH_TIMEOUT_SECONDS) {
                                state[0] = DriveState.FAILED;
                                fail(label + " timed out");
                            }
                            break;
                        case COMPLETE:
                        case FAILED:
                            break;
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
        intake.intakeOff();
        shooter.stopShooter();
        intake.stopShoot();
        intake.armUp();
    }
}
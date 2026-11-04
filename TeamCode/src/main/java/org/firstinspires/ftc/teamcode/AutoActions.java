package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class AutoActions {
    private enum CollectionState { OFF, COLLECTING, FULL }
    private enum ShotState { SPIN_UP, FEEDING, COMPLETE }

    private final Follower follower;
    private final IntakeTransferArm intake;
    private final Shooter shooter;
    private final Sensors sensors;
    private final Pose parkPose;
    private final Pose parkControl;

    private CollectionState collectionState = CollectionState.OFF;
    private boolean collectingFlower, parking;
    private String status = "Initialized";
    private Command activeRoutine;
    private double routineStartTime;

    public double ARM_SETTLE_SECONDS = 0.40;
    public double PICKUP_DWELL_SECONDS = 0.70;
    public double FEED_SECONDS = 1.50;
    public double SPIN_UP_TIMEOUT_SECONDS = 2.0;
    public double SHOT_TIMEOUT_SECONDS = 5.0;
    public double PATH_TIMEOUT_SECONDS = 10.0;
    public double PARK_DEADLINE_SECONDS = 25.0;

    public AutoActions(
            Follower follower,
            IntakeTransferArm intake,
            Shooter shooter,
            Sensors sensors,
            Pose parkPose,
            Pose parkControl) {
        this.follower = follower;
        this.intake = intake;
        this.shooter = shooter;
        this.sensors = sensors;
        this.parkPose = parkPose;
        this.parkControl = parkControl;
    }

    private static double now() {
        return System.nanoTime() * 1e-9;
    }

    public String status() {
        return status;
    }

    public void start(Command routine) {
        activeRoutine = routine;
        routineStartTime = now();
        Scheduler.schedule(routine);
    }

    /** Runs one loop of the routine, leaving for PARK at the deadline. Returns true when finished. */
    public boolean update() {
        updateCollection();
        if (!parking && now() - routineStartTime >= PARK_DEADLINE_SECONDS) {
            Scheduler.cancel(activeRoutine);
            activeRoutine = park();
            Scheduler.schedule(activeRoutine);
        }
        Scheduler.execute();
        return activeRoutine.done();
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
                    stowKeepingFlywheel();
                }),
                instant(() -> {
                    if (flower) intake.intakeFlower();
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
                        .setDone(() -> collectionState == CollectionState.FULL
                                || (now() - started[0]) >= PICKUP_DWELL_SECONDS),
                instant(() -> {
                    status = "Stow after pickup";
                    stowKeepingFlywheel();
                }),
                pause(ARM_SETTLE_SECONDS)
        );
    }

    /** Fires the held elements. A shot that cannot finish is skipped so the routine keeps going. */
    public Command shoot(boolean far, String label) {
        final double[] started = {0}, last = {0}, fed = {0};
        final ShotState[] state = {ShotState.SPIN_UP};
        final boolean[] speedCheckBypassed = {false};

        return Command.build().requiring(shooter)
                .setStart(() -> {
                    status = label;
                    stowKeepingFlywheel();
                    shooter.setCurTargetVelocity(far ? "long" : "short", 0);
                    started[0] = last[0] = now();
                    fed[0] = 0;
                    state[0] = ShotState.SPIN_UP;
                    speedCheckBypassed[0] = false;
                })
                .setExecute(() -> {
                    double time = now();
                    if (state[0] == ShotState.FEEDING) fed[0] += time - last[0];
                    last[0] = time;

                    switch (state[0]) {
                        case SPIN_UP:
                            if (shooter.shooterAtTarget()) {
                                state[0] = ShotState.FEEDING;
                            } else if (time - started[0] > SPIN_UP_TIMEOUT_SECONDS) {
                                speedCheckBypassed[0] = true;
                                status = label + " (speed check bypassed)";
                                state[0] = ShotState.FEEDING;
                            }
                            break;
                        case FEEDING:
                            if (fed[0] >= FEED_SECONDS) state[0] = ShotState.COMPLETE;
                            else if (!speedCheckBypassed[0] && !shooter.shooterAtTarget()) state[0] = ShotState.SPIN_UP;
                            break;
                        case COMPLETE:
                            break;
                    }

                    if (time - started[0] > SHOT_TIMEOUT_SECONDS && state[0] != ShotState.COMPLETE) {
                        status = label + " timed out, moving on";
                        state[0] = ShotState.COMPLETE;
                    }

                    switch (state[0]) {
                        case FEEDING:
                            intake.continueShoot();
                            intake.feedShooter();
                            break;
                        case SPIN_UP:
                            intake.intakeOff();
                            intake.stopShoot();
                            break;
                        case COMPLETE:
                            stowKeepingFlywheel();
                            break;
                    }
                })
                .setDone(() -> state[0] == ShotState.COMPLETE)
                .setEnd(reason -> stowKeepingFlywheel());
    }

    /** Follows a path. A path that times out is abandoned so the routine keeps going. */
    public Command drive(Path path, String label) {
        final double[] started = {0};

        return Command.build().requiring(follower)
                .setStart(() -> {
                    status = label;
                    started[0] = now();
                    follower.follow(path);
                })
                .setDone(() -> pathFinished(started[0], label));
    }

    /** Drives from wherever the robot is into the LOADING ZONE. */
    public Command park() {
        final double[] started = {0};

        return Command.build().requiring(follower)
                .setStart(() -> {
                    parking = true;
                    status = "Parking";
                    started[0] = now();
                    stopMechanisms();
                    follower.follow(parkPath(follower.pose()));
                })
                .setDone(() -> pathFinished(started[0], "Parking"));
    }

    private Path parkPath(Pose from) {
        Path path = distance(from, parkPose) > distance(parkControl, parkPose)
                ? Paths.curve(from, parkControl, parkPose)
                : Paths.line(from, parkPose);
        return path.heading(linearHeading(from, parkPose));
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    private boolean pathFinished(double startedAt, String label) {
        if (!follower.following()) return true;
        if (now() - startedAt > PATH_TIMEOUT_SECONDS) {
            status = label + " timed out, moving on";
            follower.hold(follower.pose());
            return true;
        }
        return false;
    }

    private void stowKeepingFlywheel() {
        collectionState = CollectionState.OFF;
        intake.intakeOff();
        intake.stopShoot();
        intake.armUp();
    }

    public void stopMechanisms() {
        stowKeepingFlywheel();
        shooter.stopShooter();
    }
}

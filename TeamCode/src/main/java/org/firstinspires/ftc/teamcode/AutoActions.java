package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class AutoActions {
    private enum CollectionState { OFF, COLLECTING, FULL }
    private enum ActionType { NONE, DRIVE, PREPARE_PICKUP, FINISH_PICKUP, SHOOT, STOP }
    private enum ActionState {
        IDLE,
        FOLLOWING,
        PREPARING_PICKUP,
        WAITING_FOR_PICKUP,
        SETTLING,
        SPIN_UP,
        FEEDING,
        COMPLETE,
        FAILED
    }

    private final Follower follower;
    private final IntakeTransferArm intake;
    private final Shooter shooter;
    private final Sensors sensors;

    private CollectionState collectionState = CollectionState.OFF;
    private ActionType actionType = ActionType.NONE;
    private ActionState actionState = ActionState.IDLE;
    private boolean collectingFlower;
    private boolean failed;
    private String status = "Initialized";
    private String activeLabel = "";
    private double actionStarted;
    private double lastTick;
    private double fedSeconds;

    public double ARM_SETTLE_SECONDS = 0.40;
    public double PICKUP_DWELL_SECONDS = 0.70;
    public double FEED_SECONDS = 1.50;
    public double SHOT_TIMEOUT_SECONDS = 5.0;
    public double PATH_TIMEOUT_SECONDS = 10.0;

    public AutoActions(
            Follower follower,
            BioBuzzIntake intake,
            BioBuzzShooter shooter,
            BioBuzzShotStopper shotStopper,
            BioBuzzFlowerArm arm,
            BioBuzzBallSensors sensors) {
        this.follower = follower;
        this.intake = intake;
        this.shooter = shooter;
        this.shotStopper = shotStopper;
        this.arm = arm;
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

    public boolean actionComplete() {
        return actionState == ActionState.COMPLETE || actionState == ActionState.FAILED;
    }

    public static Interpolator linearHeading(Pose from, Pose to) {
        double start = from.heading();
        double change = Angle.error(start, to.heading());
        return (curve, t) -> {
            double progress = curve.length() > 0
                    ? 1.0 - curve.remainingDistance(t) / curve.length()
                    : t;
            progress = Math.max(0.0, Math.min(1.0, progress));
            return Angle.normalize(start + change * progress);
        };
    }

    public void startDrive(Path path, String label) {
        if (failed) return;
        actionType = ActionType.DRIVE;
        actionState = ActionState.FOLLOWING;
        activeLabel = label;
        status = label;
        follower.follow(path);
    }

    public void stopMechanisms() {
        actionType = ActionType.STOP;
        actionState = ActionState.IDLE;
        status = "Stopped";
        intake.intakeOff();
        shooter.stopShooter();
        if (shotStopper != null) shotStopper.block();
        if (arm != null) arm.armUp();
    }

    public void updateCollection() {
        switch (collectionState) {
            case OFF:
                return;
            case COLLECTING:
                if (sensors.isFull()) {
                    collectionState = CollectionState.FULL;
                }
                break;
            case FULL:
                break;
        }

        if (collectionState == CollectionState.COLLECTING) {
            intake.intakeOn();
        } else if (collectionState == CollectionState.FULL) {
            intake.intakeOff();
            if (collectingFlower) {
                if (arm != null) arm.armBlock();
            }
        }
    }

    public Command preparePickup(boolean flower) {
        return sequential(
                instant(() -> {
                    status = "Prepare pickup";
                    stopMechanisms();
                    collectingFlower = flower;
                    collectionState = CollectionState.COLLECTING;
                    actionState = ActionState.PREPARING_PICKUP;
                    actionType = ActionType.PREPARE_PICKUP;
                }),
                instant(() -> {
                    if (flower) {
                        if (arm != null) arm.armBlock();
                    } else {
                        if (arm != null) arm.armUp();
                    }
                }),
                pause(ARM_SETTLE_SECONDS),
                instant(() -> {
                    status = "Collecting";
                    actionState = ActionState.WAITING_FOR_PICKUP;
                    updateCollection();
                })
        );
    }

    public Command finishPickup() {
        return sequential(
                Command.build().setStart(() -> actionStarted = now())
                        .setDone(() -> !failed && (collectionState == CollectionState.FULL || (now() - actionStarted) >= PICKUP_DWELL_SECONDS)),
                instant(() -> {
                    status = "Stow after pickup";
                    stopMechanisms();
                    actionState = ActionState.SETTLING;
                }),
                pause(ARM_SETTLE_SECONDS)
        );
    }

    public Command shoot(boolean far, String label) {
        final double[] started = {0};
        final double[] last = {0};
        final double[] fed = {0};
        final ActionState[] state = {ActionState.SPIN_UP};

        return Command.build().requiring(shooter)
                .setStart(() -> {
                    status = label;
                    actionState = ActionState.SPIN_UP;
                    actionType = ActionType.SHOOT;
                    actionStarted = now();
                    started[0] = now();
                    last[0] = now();
                    fed[0] = 0;
                    state[0] = ActionState.SPIN_UP;
                    stopMechanisms();
                    shooter.setCurTargetVelocity(far ? "long" : "short", 0);
                })
                .setExecute(() -> {
                    double time = now();
                    if (state[0] == ActionState.FEEDING) {
                        fed[0] += time - last[0];
                    }
                    last[0] = time;

                    switch (state[0]) {
                        case SPIN_UP:
                            if (shooter.shooterAtTarget()) {
                                state[0] = ActionState.FEEDING;
                            }
                            break;
                        case FEEDING:
                            if (fed[0] >= FEED_SECONDS) {
                                state[0] = ActionState.COMPLETE;
                            } else if (!shooter.shooterAtTarget()) {
                                state[0] = ActionState.SPIN_UP;
                            }
                            break;
                        case COMPLETE:
                        case FAILED:
                            break;
                        default:
                            break;
                    }

                    if (time - started[0] > SHOT_TIMEOUT_SECONDS && state[0] != ActionState.COMPLETE) {
                        state[0] = ActionState.FAILED;
                        fail(label + " timed out");
                    }

                    switch (state[0]) {
                        case FEEDING:
                            intake.feedShooter();
                            if (shotStopper != null) shotStopper.open();
                            break;
                        case SPIN_UP:
                            intake.intakeOff();
                            if (shotStopper != null) shotStopper.block();
                            break;
                        case COMPLETE:
                        case FAILED:
                            stopMechanisms();
                            break;
                        default:
                            break;
                    }
                })
                .setDone(() -> !failed && state[0] == ActionState.COMPLETE)
                .setEnd(reason -> stopMechanisms());
    }

    public Command drive(Path path, String label) {
        final double[] started = {0};
        final ActionState[] state = {ActionState.FOLLOWING};

        return Command.build().requiring(follower)
                .setStart(() -> {
                    status = label;
                    started[0] = now();
                    state[0] = ActionState.FOLLOWING;
                    actionType = ActionType.DRIVE;
                    actionState = ActionState.FOLLOWING;
                    follower.follow(path);
                })
                .setDone(() -> {
                    switch (state[0]) {
                        case FOLLOWING:
                            if (failed) {
                                state[0] = ActionState.FAILED;
                            } else if (!follower.following()) {
                                state[0] = ActionState.COMPLETE;
                            } else if ((now() - started[0]) > PATH_TIMEOUT_SECONDS) {
                                state[0] = ActionState.FAILED;
                                fail(label + " timed out");
                            }
                            break;
                        case COMPLETE:
                        case FAILED:
                            break;
                        default:
                            break;
                    }
                    return state[0] == ActionState.COMPLETE && !failed;
                });
    }

    private Command pause(double seconds) {
        final double[] started = {0};
        return Command.build().setStart(() -> started[0] = now())
                .setDone(() -> (now() - started[0]) >= seconds);
    }

    private void fail(String message) {
        failed = true;
        status = message;
        stopMechanisms();
    }
}

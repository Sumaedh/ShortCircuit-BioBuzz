package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;

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
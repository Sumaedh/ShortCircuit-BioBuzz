package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

/** Mechanism actions and endpoint checks shared by all four autonomous files. */
public final class BioBuzzAutoActions {
    private final BioBuzzMechanisms robot;
    private final Follower follower;
    private String status = "Initialized";
    public BioBuzzAutoActions(BioBuzzMechanisms robot, Follower follower) {
        this.robot = robot; this.follower = follower;
    }
    public String status() { return status; }


    public static Interpolator linearHeading(Pose from, Pose to) {
        final double start = from.heading();
        final double change = Angle.error(start, to.heading());
        return (curve, t) -> {
            double length = curve.length();
            double progress = length > 0 ? 1.0 - curve.remainingDistance(t) / length : t;
            progress = Math.max(0.0, Math.min(1.0, progress));
            return Angle.normalize(start + change * progress);
        };
    }


    public Command preparePickup(boolean flower) {
        return sequential(
                instant(() -> {
                    status = flower ? "Lower flower arm" : "Prepare corner pickup";
                    robot.endFeedParametric();
                    robot.shooter.shooterOffParametric();
                    if (flower) robot.flowerArm.armDownParametric();
                    else robot.flowerArm.armUpParametric();
                }),
                pause(BioBuzzConfig.ARM_SETTLE_SECONDS),
                instant(() -> { status = "Collecting"; robot.beginCollectParametric(); })
        );
    }
    public Command finishPickup() {
        final double[] start = {0};
        Command dwell = Command.build()
                .setStart(() -> start[0] = BioBuzzConfig.now())
                .setDone(() -> robot.collectionFull()
                        || BioBuzzConfig.now() - start[0] >= BioBuzzConfig.PICKUP_DWELL_SECONDS);
        return sequential(dwell,
                instant(() -> {
                    status = "Stop intake and stow arm";
                    robot.endCollectParametric();
                    robot.flowerArm.armUpParametric();
                }), pause(BioBuzzConfig.ARM_SETTLE_SECONDS));
    }
    public Command pause(double seconds) {
        final double[] start = {0};
        return Command.build().setStart(() -> start[0] = BioBuzzConfig.now())
                .setDone(() -> BioBuzzConfig.now() - start[0] >= seconds);
    }

    public Command shoot(boolean far, String label) {
        final double[] start = {0}, previous = {0}, fedSeconds = {0};
        return Command.build()
                .requiring(robot)
                .setStart(() -> {
                    status = label + ": spin up";
                    robot.endCollectParametric(); robot.endFeedParametric();
                    if (far) robot.shooter.shooterFarParametric();
                    else robot.shooter.shooterCloseParametric();
                    start[0] = previous[0] = BioBuzzConfig.now(); fedSeconds[0] = 0;
                })
                .setExecute(() -> {
                    double now = BioBuzzConfig.now();
                    if (robot.shooter.isReady()) {
                        status = label + ": feed";
                        if (robot.intake.getPower() > 0) fedSeconds[0] += now - previous[0];
                        robot.beginFeedParametric();
                    } else {
                        status = label + ": waiting for speed";
                        robot.endFeedParametric();
                    }
                    previous[0] = now;
                    if (now - start[0] > BioBuzzConfig.AUTO_SHOT_TIMEOUT_SECONDS
                            && fedSeconds[0] < BioBuzzConfig.AUTO_FEED_SECONDS) {
                        throw new IllegalStateException(label + " timed out before completing the feed window");
                    }
                })
                .setDone(() -> fedSeconds[0] >= BioBuzzConfig.AUTO_FEED_SECONDS)
                .setEnd(reason -> { robot.endFeedParametric(); robot.shooter.shooterOffParametric(); });
    }


    public Command drive(Path path, String label) {
        final double[] start = {0}, settledSince = {Double.NaN};
        final Pose end = path.endPose();
        return Command.build().requiring(follower)
                .setStart(() -> {
                    status = label; start[0] = BioBuzzConfig.now(); settledSince[0] = Double.NaN;
                    follower.follow(path);
                })
                .setDone(() -> {
                    double now = BioBuzzConfig.now();
                    Pose actual = follower.pose();
                    double angle = Math.atan2(Math.sin(end.heading() - actual.heading()),
                            Math.cos(end.heading() - actual.heading()));
                    boolean settled = !follower.following()
                            && Math.hypot(end.x() - actual.x(), end.y() - actual.y()) <= BioBuzzConfig.AUTO_POSITION_TOLERANCE_INCHES
                            && Math.abs(Math.toDegrees(angle)) <= BioBuzzConfig.AUTO_HEADING_TOLERANCE_DEGREES;
                    if (!settled) settledSince[0] = Double.NaN;
                    else if (Double.isNaN(settledSince[0])) settledSince[0] = now;
                    if (settled && now - settledSince[0] >= BioBuzzConfig.AUTO_POSE_SETTLE_SECONDS) return true;
                    if (now - start[0] > BioBuzzConfig.AUTO_PATH_TIMEOUT_SECONDS) {
                        throw new IllegalStateException(label + " timed out before reaching its endpoint");
                    }
                    return false;
                });
    }
}
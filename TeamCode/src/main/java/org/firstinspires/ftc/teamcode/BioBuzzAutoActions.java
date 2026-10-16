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
    private final Follower follower;
    private final BioBuzzIntake intake;
    private final BioBuzzShooter shooter;
    private final BioBuzzShotStopper shotStopper;
    private final BioBuzzIntakeStopper intakeStopper;
    private final BioBuzzFlowerArm arm;
    private final BioBuzzBallSensors sensors;
    private boolean collecting, fullLatched, failed;
    private String status = "Initialized";

    public double ARM_SETTLE_SECONDS = 0.40;
    public double PICKUP_DWELL_SECONDS = 0.70;
    public double FEED_SECONDS = 1.50;
    public double SHOT_TIMEOUT_SECONDS = 5.0;
    public double PATH_TIMEOUT_SECONDS = 10.0;

    public BioBuzzAutoActions(Follower follower, BioBuzzIntake intake, BioBuzzShooter shooter,
                              BioBuzzShotStopper shotStopper, BioBuzzIntakeStopper intakeStopper,
                              BioBuzzFlowerArm arm, BioBuzzBallSensors sensors) {
        this.follower = follower; this.intake = intake; this.shooter = shooter;
        this.shotStopper = shotStopper; this.intakeStopper = intakeStopper;
        this.arm = arm; this.sensors = sensors;
    }

    private static double now() { return System.nanoTime() * 1e-9; }
    public String status() { return status; }
    public boolean failed() { return failed; }

    public static Interpolator linearHeading(Pose from, Pose to) {
        double start = from.heading();
        double change = Angle.error(start, to.heading());
        return (curve, t) -> {
            double progress = curve.length() > 0 ? 1.0 - curve.remainingDistance(t) / curve.length() : t;
            return Angle.normalize(start + change * Math.max(0.0, Math.min(1.0, progress)));
        };
    }

    public Command pause(double seconds) {
        final double[] started = {0};
        return Command.build().setStart(() -> started[0] = now())
                .setDone(() -> now() - started[0] >= seconds);
    }

    public Command preparePickup(boolean flower) {
        return sequential(
                instant(() -> { status = "Prepare pickup"; stopMechanisms(); }),
                instant(flower ? arm.armDownParametric() : arm.armUpParametric()),
                pause(ARM_SETTLE_SECONDS),
                instant(() -> {
                    status = "Collecting"; collecting = true; fullLatched = false;
                    updateCollection();
                }));
    }

    public void updateCollection() {
        if (!collecting) return;
        if (sensors.isFull()) fullLatched = true;
        if (fullLatched) { intake.intakeOff(); intakeStopper.block(); }
        else { intakeStopper.open(); intake.intakeOn(); }
    }

    public Command finishPickup() {
        final double[] started = {0};
        return sequential(
                Command.build().setStart(() -> started[0] = now())
                        .setDone(() -> fullLatched || now() - started[0] >= PICKUP_DWELL_SECONDS),
                instant(() -> { status = "Stow after pickup"; stopMechanisms(); }),
                pause(ARM_SETTLE_SECONDS));
    }

    public Command shoot(boolean far, String label) {
        final double[] started = {0}, last = {0}, fed = {0};
        final boolean[] feeding = {false};
        return Command.build().requiring(shooter, intake)
                .setStart(() -> {
                    status = label; stopMechanisms();
                    shooter.setCurTargetVelocity(far ? "long" : "short", 0);
                    started[0] = last[0] = now(); fed[0] = 0; feeding[0] = false;
                })
                .setExecute(() -> {
                    double time = now();
                    if (feeding[0]) fed[0] += time - last[0];
                    last[0] = time;
                    feeding[0] = shooter.shooterAtTarget();
                    if (feeding[0]) { shotStopper.open(); intake.feedShooter(); }
                    else { shotStopper.block(); intake.intakeOff(); }
                    if (time - started[0] > SHOT_TIMEOUT_SECONDS && fed[0] < FEED_SECONDS) {
                        failed = true; status = label + " timed out"; stopMechanisms();
                    }
                })
                .setDone(() -> !failed && fed[0] >= FEED_SECONDS)
                .setEnd(reason -> stopMechanisms());
    }

    public Command drive(Path path, String label) {
        final double[] started = {0};
        return Command.build().requiring(follower)
                .setStart(() -> { status = label; started[0] = now(); follower.follow(path); })
                .setDone(() -> {
                    if (failed) return false;
                    if (!follower.following()) return true;
                    if (now() - started[0] > PATH_TIMEOUT_SECONDS) {
                        failed = true; status = label + " timed out"; stopMechanisms();
                    }
                    return false;
                });
    }

    public void stopMechanisms() {
        collecting = false;
        intake.intakeOff(); shooter.stopShooter();
        shotStopper.block(); intakeStopper.block(); arm.armUp();
    }
}
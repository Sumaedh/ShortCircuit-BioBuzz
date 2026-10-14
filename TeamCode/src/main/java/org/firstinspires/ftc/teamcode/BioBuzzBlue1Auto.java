package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import java.util.Objects;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

/** Blue 1: original route geometry with shooting and pickup actions. */
@Autonomous(name = "BIOBUZZ Blue 1", group = "BIOBUZZ")
public class BioBuzzBlue1Auto extends LinearOpMode {
    private Follower follower;
    private final BioBuzzMechanisms robot = new BioBuzzMechanisms();
    private BioBuzzAutoActions actions;

    // Provisional profile choices: select close/far after measuring shot distances.
    private static final boolean INITIAL_SHOT_FAR = false;
    private static final boolean MIDDLE_SHOT_FAR = false;
    private static final boolean FINAL_SHOT_FAR = false;

    // Inputs are inches/degrees. Red retains the existing 180-degree field rotation.
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(84.1051, 132.7, 270);
    private final Pose sideApproach = poseFactory.of(125.5152, 94.1156, 0);
    private final Pose sideControl = poseFactory.of(103.8315, 114.1313, 0);
    private final Pose sidePickup = poseFactory.of(130.7, 94, 0);
    private final Pose middleShot = poseFactory.of(81.4519, 23.2505, 90);
    private final Pose middleControl = poseFactory.of(102.1051, 25.8104, 0);
    private final Pose endApproach = poseFactory.of(94.3797, 14.0527, 270);
    private final Pose endControl = poseFactory.of(97.7156, 27.3965, 0);
    private final Pose endPickup = poseFactory.of(94.3797, 11, 270);
    private final Pose finalShot = poseFactory.of(84.1051, 132.7, 270);
    private final Pose returnControl = poseFactory.of(141.723, 54.7839, 0);

    // Nothing here executes while the sequence is being constructed.
    public Command autoRoutine() {
        return sequential(
                actions.shoot(INITIAL_SHOT_FAR, "Initial shot"),
                actions.drive(path1(), "Green approach"),
                actions.preparePickup(true),
                actions.drive(path2(), "Straight first pickup"),
                actions.finishPickup(),
                actions.drive(path3(), "Indigo return"),
                actions.shoot(MIDDLE_SHOT_FAR, "Middle shot"),
                actions.drive(path4(), "Pink approach"),
                actions.preparePickup(true),
                actions.drive(path5(), "Straight end-flower pickup"),
                actions.finishPickup(),
                actions.drive(path6(), "Brown return"),
                actions.shoot(FINAL_SHOT_FAR, "Final shot"),
                instant(() -> robot.stopAllParametric())
        );
    }

    @Override public void runOpMode() {
        Scheduler.reset();
        // Keep the drafts buildable, but do not run a guessed robot configuration.
        try { BioBuzzConfig.validateAuto(); }
        catch (IllegalStateException ex) {
            telemetry.addLine(ex.getMessage()); telemetry.update();
            waitForStart();
            return;
        }

        try {
            robot.init(hardwareMap);
            // YOUR configured Pedro 3 factory; no localizer is invented in this package.
            follower = Objects.requireNonNull(Constants.create(hardwareMap),
                    "Configure your Pedro Constants.create() before running autonomous");
            follower.setPose(startPose);
            follower.update();
            actions = new BioBuzzAutoActions(robot, follower);
            Command routine = autoRoutine();
            telemetry.addLine("Initialized; place robot at the specified start pose");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            double startedAt = BioBuzzConfig.now();
            schedule(routine);
            while (opModeIsActive()) {
                double now = BioBuzzConfig.now();
                follower.update();
                robot.update(now);
                Scheduler.execute();
                telemetry.addData("Auto action", actions.status());
                telemetry.addData("X / Y", follower.pose().x() + " / " + follower.pose().y());
                telemetry.addData("Heading degrees", Math.toDegrees(follower.pose().heading()));
                robot.telemetry(telemetry);
                telemetry.update();
                if (routine.done()) break;
                if (now - startedAt > BioBuzzConfig.AUTO_WATCHDOG_SECONDS) {
                    throw new IllegalStateException("Autonomous watchdog expired");
                }
                idle();
            }
        } catch (RuntimeException ex) {
            telemetry.addData("AUTO STOPPED", ex.getMessage()); telemetry.update();
            throw ex;
        } finally {
            Scheduler.reset();
            robot.stopAll();
            if (follower != null) { follower.stop(); follower.drivetrain.stop(); }
        }
    }

    // GREEN: start to side-flower approach.
    public Path path1() {
        return Paths.curve(startPose, sideControl, sideApproach).heading(BioBuzzAutoActions.linearHeading(startPose, sideApproach));
    }

    // Straight side-flower entry.
    public Path path2() {
        return Paths.line(sideApproach, sidePickup).heading(BioBuzzAutoActions.linearHeading(sideApproach, sidePickup));
    }

    // INDIGO: side pickup to middle shot.
    public Path path3() {
        return Paths.curve(sidePickup, middleControl, middleShot).heading(BioBuzzAutoActions.linearHeading(sidePickup, middleShot));
    }

    // PINK: middle shot to end-flower approach.
    public Path path4() {
        return Paths.curve(middleShot, endControl, endApproach).heading(BioBuzzAutoActions.linearHeading(middleShot, endApproach));
    }

    // Straight end-flower entry.
    public Path path5() {
        return Paths.line(endApproach, endPickup).heading(BioBuzzAutoActions.linearHeading(endApproach, endPickup));
    }

    // BROWN: end pickup to final shot.
    public Path path6() {
        return Paths.curve(endPickup, returnControl, finalShot).heading(BioBuzzAutoActions.linearHeading(endPickup, finalShot));
    }
}
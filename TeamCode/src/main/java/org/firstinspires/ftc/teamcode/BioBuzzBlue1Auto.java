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

// This import must match the package of YOUR Constants.java.
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "BIOBUZZ Blue 1 Paths", group = "Autonomous")
public class BioBuzzBlue1Auto extends LinearOpMode {

    // Controls the drivetrain and tracks the robot's position.
    private Follower follower;

    // Pose inputs use inches for X/Y and degrees for heading.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Starting position and heading of the robot's center.
    private final Pose startPose =
            poseFactory.of(84.1051, 132.7, 270);

    // Position just before the side-flower pickup.
    private final Pose sideApproach =
            poseFactory.of(125.5152, 94.1156, 0);

    // Shapes the first curve; it is not a stop.
    private final Pose sideControl =
            poseFactory.of(103.8315, 114.1313, 0);

    // End of the short straight side-flower approach.
    private final Pose sidePickup =
            poseFactory.of(130.7, 94, 0);

    // Destination for the middle shooting position.
    private final Pose middleShot =
            poseFactory.of(81.4519, 23.2505, 90);

    // Shapes the curve toward the middle shooting position.
    private final Pose middleControl =
            poseFactory.of(102.1051, 25.8104, 0);

    // Position just before the end-flower pickup.
    private final Pose endApproach =
            poseFactory.of(94.3797, 14.0527, 270);

    // Shapes the curve toward the end flower.
    private final Pose endControl =
            poseFactory.of(97.7156, 27.3965, 0);

    // End of the straight end-flower approach.
    private final Pose endPickup =
            poseFactory.of(94.3797, 11, 270);

    // Final driving destination.
    private final Pose finalShot =
            poseFactory.of(84.1051, 132.7, 270);

    // Shapes the return curve; the robot need not pass through it.
    private final Pose returnControl =
            poseFactory.of(141.723, 54.7839, 0);

    // Follows the six paths in order.
    // This routine only drives; it does not operate intake or shooter.
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
        );
    }

    @Override
    public void runOpMode() {

        // Clear any previous scheduler state.
        Scheduler.reset();

        // Create the follower using your robot's configuration.
        follower = Constants.create(hardwareMap);

        // Set the initial position estimate.
        follower.setPose(startPose);
        follower.update();

        try {
            // Wait for START on the Driver Station.
            waitForStart();

            if (isStopRequested()) {
                return;
            }

            // Schedule the routine once.
            schedule(autoRoutine());

            while (opModeIsActive()) {

                // Update localization and drivetrain control.
                follower.update();

                // Advance the sequence as each path completes.
                Scheduler.execute();

                // Display the estimated robot position.
                telemetry.addData("X", follower.pose().x());
                telemetry.addData("Y", follower.pose().y());
                telemetry.addData(
                        "Heading (degrees)",
                        Math.toDegrees(follower.pose().heading())
                );
                telemetry.update();

                idle();
            }

        } finally {
            // Stop scheduled commands and drivetrain output on exit.
            Scheduler.reset();
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    // GREEN: curve from the start to the side-flower approach.
    // linear() changes heading along the curve; it does not straighten it.
    public Path path1() {
        return Paths.curve(
                startPose,
                sideControl,
                sideApproach
        ).linear(startPose, sideApproach);
    }

    // Short straight approach to the side-flower pickup.
    public Path path2() {
        return Paths.line(
                sideApproach,
                sidePickup
        ).linear(sideApproach, sidePickup);
    }

    // INDIGO: curve from the side pickup to the middle shot position.
    public Path path3() {
        return Paths.curve(
                sidePickup,
                middleControl,
                middleShot
        ).linear(sidePickup, middleShot);
    }

    // PINK: curve from the middle shot position to the end-flower approach.
    public Path path4() {
        return Paths.curve(
                middleShot,
                endControl,
                endApproach
        ).linear(middleShot, endApproach);
    }

    // Straight approach to the end-flower pickup.
    public Path path5() {
        return Paths.line(
                endApproach,
                endPickup
        ).linear(endApproach, endPickup);
    }

    // BROWN: return curve to the final shot position.
    public Path path6() {
        return Paths.curve(
                endPickup,
                returnControl,
                finalShot
        ).linear(endPickup, finalShot);
    }
}
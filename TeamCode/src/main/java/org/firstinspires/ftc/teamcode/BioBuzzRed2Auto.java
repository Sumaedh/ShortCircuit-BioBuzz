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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "BIOBUZZ Red 2 Paths", group = "Autonomous")
public class BioBuzzRed2Auto extends LinearOpMode {

    // Controls driving and tracks the robot's estimated position.
    private Follower follower;

    // Rotate the blue planning coordinates 180 degrees for red.
    // Actual coordinates become (141.5 - X, 141.5 - Y).
    // Headings and control points are transformed automatically.
    private final PoseFactory poseFactory = PoseFactory.degrees()
            .mirrorX(70.75)
            .mirrorY(70.75);

    // After transformation: X = 33.3949, Y = 8.8, heading = 90.
    // This is 24 inches toward -X from the Red 1 starting position.
    private final Pose startPose =
            poseFactory.of(108.1051, 132.7, 270);

    // Align before driving straight toward the corner pickup.
    private final Pose cornerApproach =
            poseFactory.of(127.5, 113.5, 90);

    // These control points shape the first curve, not extra stops.
    private final Pose cornerControl1 =
            poseFactory.of(108.1051, 107.5, 0);
    private final Pose cornerControl2 =
            poseFactory.of(127.5, 105.5, 0);

    // Corner pickup is 15 inches ahead of cornerApproach.
    private final Pose cornerPickup =
            poseFactory.of(127.5, 128.5, 90);

    // Middle shooting destination and its approach control point.
    private final Pose middleShot =
            poseFactory.of(81.4519, 23.2505, 90);
    private final Pose middleControl =
            poseFactory.of(102.1051, 25.8104, 0);

    // Approach and pickup coordinates for the end flower.
    private final Pose endApproach =
            poseFactory.of(94.3797, 14.0527, 270);
    private final Pose endControl =
            poseFactory.of(97.7156, 27.3965, 0);
    private final Pose endPickup =
            poseFactory.of(94.3797, 11, 270);

    // Final destination and control point shaping the return curve.
    private final Pose finalShot =
            poseFactory.of(84.1051, 132.7, 270);
    private final Pose returnControl =
            poseFactory.of(141.723, 54.7839, 0);

    // Run all seven driving paths in order.
    // No intake, shooting, or shooting pauses are included.
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Use your robot's configured follower.
        // Report a clear initialization error if create() returns null.
        follower = Objects.requireNonNull(
                Constants.create(hardwareMap),
                "Constants.create(hardwareMap) returned null"
        );

        try {
            // Set the transformed red starting position.
            follower.setPose(startPose);
            follower.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            // Start the sequence once.
            schedule(autoRoutine());

            while (opModeIsActive()) {
                // Update driving, then advance the command sequence.
                follower.update();
                Scheduler.execute();

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
            // Clear commands and stop the drivetrain when exiting.
            Scheduler.reset();
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    // GREEN: curve from the shifted start to the corner alignment.
    // linear() interpolates heading along the curved driving path.
    public Path path1() {
        return Paths.curve(
                startPose,
                cornerControl1,
                cornerControl2,
                cornerApproach
        ).linear(startPose, cornerApproach);
    }

    // Drive straight 15 inches into the corner pickup.
    public Path path2() {
        return Paths.line(
                cornerApproach,
                cornerPickup
        ).linear(cornerApproach, cornerPickup);
    }

    // Back out straight while keeping the same robot heading.
    public Path path3() {
        return Paths.line(
                cornerPickup,
                cornerApproach
        ).linear(cornerPickup, cornerApproach);
    }

    // INDIGO: travel from the corner approach to the middle position.
    public Path path4() {
        return Paths.curve(
                cornerApproach,
                middleControl,
                middleShot
        ).linear(cornerApproach, middleShot);
    }

    // PINK: travel to the end-flower alignment position.
    public Path path5() {
        return Paths.curve(
                middleShot,
                endControl,
                endApproach
        ).linear(middleShot, endApproach);
    }

    // Drive straight into the end-flower pickup.
    public Path path6() {
        return Paths.line(
                endApproach,
                endPickup
        ).linear(endApproach, endPickup);
    }

    // BROWN: return to the final position.
    public Path path7() {
        return Paths.curve(
                endPickup,
                returnControl,
                finalShot
        ).linear(endPickup, finalShot);
    }
}
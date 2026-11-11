package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BulkCaching;
import org.firstinspires.ftc.teamcode.IntakeTransferArm;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Shooter;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "BIOBUZZ Blue 2", group = "BIOBUZZ")
@SuppressWarnings("unused")
public class Blue2Auto extends LinearOpMode {
    private Follower follower;
    private final IntakeTransferArm in = new IntakeTransferArm();
    private final Shooter sh = new Shooter();
    private final Sensors sensors = new Sensors();
    private String status = "Waiting for start";

    private static final double AUTO_WATCHDOG_SECONDS = 30.0;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(84.1, 129.5, 270);
    private final Pose startShot = startPose;

    private final Pose sideControl1 = poseFactory.of(93.5, 117.5, 0);
    private final Pose sideControl2 = poseFactory.of(107.5, 99.5, 0);
    private final Pose sideApproach = poseFactory.of(118.0, 95.5, 0);
    private final Pose sidePickup = poseFactory.of(127.3, 95.5, 0);

    private final Pose farShotControl1 = poseFactory.of(117.5, 83.5, 0);
    private final Pose farShotControl2 = poseFactory.of(117.5, 27.5, 0);
    private final Pose farShot = poseFactory.of(84.1, 13.5, 90);

    private final Pose endControl1 = poseFactory.of(84.1, 22.0, 0);
    private final Pose endControl2 = poseFactory.of(88.0, 30.0, 0);
    private final Pose endApproach = poseFactory.of(93.0, 27.5, 270);
    private final Pose endPickup = poseFactory.of(93.0, 14.2, 270);

    private final Pose gardenControl1 = poseFactory.of(119.5, 25.5, 0);
    private final Pose gardenControl2 = poseFactory.of(115.5, 117.5, 0);
    private final Pose gardenStart = poseFactory.of(108.0, 131.75, 0);
    private final Pose gardenEnd = poseFactory.of(131.0, 131.75, 0);

    private final Pose returnControl1 = poseFactory.of(120.0, 118.0, 0);
    private final Pose returnControl2 = poseFactory.of(96.0, 118.0, 0);
    private final Pose finalShot = startPose;

    private final Pose parkPose = poseFactory.of(130.8, 40.3, 0);
    private final Pose parkControl = poseFactory.of(121, 101, 0);

    // Shooting and intake lived in AutoActions, which is not in this project,
    // so this routine only drives. The comments mark where each mechanism step goes.
    public Command autoRoutine() {
        return sequential(
                // shoot preloads here
                drive(toSideFlower(), "To side flower"),
                // start flower intake here
                drive(intakeSideFlower(), "Straight side flower intake"),
                // stop intake here
                drive(toFarShot(), "To far shot"),
                // shoot here
                drive(toEndFlower(), "To end flower"),
                // start flower intake here
                drive(intakeEndFlower(), "Straight end flower intake"),
                // stop intake here
                drive(backOutOfEndFlower(), "Back out of end flower"),
                drive(backToFarShot(), "Back to far shot"),
                // shoot here
                drive(toGarden(), "To garden"),
                // start floor intake here
                drive(intakeGarden(), "Straight garden intake"),
                // stop intake here
                drive(toFinalShot(), "To final shot"),
                // shoot here
                drive(toPark(), "Park"),
                instant(() -> status = "Done")
        );
    }

    private Command drive(Path path, String name) {
        return sequential(
                instant(() -> status = name),
                follow(follower, path)
        );
    }

    @Override public void runOpMode() {
        Scheduler.reset();
        try {
            BulkCaching.enableAuto(hardwareMap);
            in.initIntakeTransfer(hardwareMap);
            sh.initShooter(hardwareMap);
            sensors.initSensors(hardwareMap);

            follower = Constants.create(hardwareMap);
            if (follower == null) {
                telemetry.addLine(Constants.NOT_READY);
                telemetry.update();
                while (!isStopRequested()) sleep(50);
                return;
            }
            follower.setPose(startPose);
            follower.update();

            Command routine = autoRoutine();
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            double startTime = System.nanoTime() * 1e-9;
            schedule(routine);
            while (opModeIsActive()) {
                follower.update();
                sh.PIDFShootingLoop();
                Scheduler.execute();
                telemetry.addData("Action", status);
                telemetry.addData("X / Y", follower.pose().x() + " / " + follower.pose().y());
                telemetry.addData("Heading degrees", Math.toDegrees(follower.pose().heading()));
                telemetry.addData("Shooter target", sh.curTargetVelocity);
                telemetry.addData("Shooter left / right", sh.getVelocity() + " / " + sh.getVelocity2());
                telemetry.addData("Storage full", sensors.lastFull());
                telemetry.update();
                if ((System.nanoTime() * 1e-9) - startTime > AUTO_WATCHDOG_SECONDS) {
                    telemetry.addLine("Autonomous time limit reached");
                    telemetry.update();
                    break;
                }
                idle();
            }
        } finally {
            Scheduler.reset();
            in.intakeOff();
            sh.stopShooter();
            in.stopShoot();
            in.armUp();
            if (follower != null) { follower.stop(); follower.drivetrain.stop(); }
        }
    }

    public Path toSideFlower() {
        return Paths.curve(startShot, sideControl1, sideControl2, sideApproach)
                .linear(startShot, sideApproach);
    }

    public Path intakeSideFlower() {
        return Paths.line(sideApproach, sidePickup)
                .linear(sideApproach, sidePickup);
    }

    public Path toFarShot() {
        return Paths.curve(sidePickup, farShotControl1, farShotControl2, farShot)
                .linear(sidePickup, farShot);
    }

    public Path toEndFlower() {
        return Paths.curve(farShot, endControl1, endControl2, endApproach)
                .linear(farShot, endApproach);
    }

    public Path intakeEndFlower() {
        return Paths.line(endApproach, endPickup)
                .linear(endApproach, endPickup);
    }

    public Path backOutOfEndFlower() {
        return Paths.line(endPickup, endApproach)
                .linear(endPickup, endApproach);
    }

    public Path backToFarShot() {
        return Paths.curve(endApproach, endControl2, endControl1, farShot)
                .linear(endApproach, farShot);
    }

    public Path toGarden() {
        return Paths.curve(farShot, gardenControl1, gardenControl2, gardenStart)
                .linear(farShot, gardenStart);
    }

    public Path intakeGarden() {
        return Paths.line(gardenStart, gardenEnd)
                .linear(gardenStart, gardenEnd);
    }

    public Path toFinalShot() {
        return Paths.curve(gardenEnd, returnControl1, returnControl2, finalShot)
                .linear(gardenEnd, finalShot);
    }

    public Path toPark() {
        return Paths.curve(finalShot, parkControl, parkPose)
                .linear(finalShot, parkPose);
    }
}
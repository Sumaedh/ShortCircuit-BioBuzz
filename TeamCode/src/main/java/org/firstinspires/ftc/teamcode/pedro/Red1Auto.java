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

import org.firstinspires.ftc.teamcode.AutoActions;
import org.firstinspires.ftc.teamcode.IntakeTransferArm;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Shooter;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous(name = "BIOBUZZ Red 1", group = "BIOBUZZ")
public class Red1Auto extends LinearOpMode {
    private Follower follower;
    private final IntakeTransferArm in = new IntakeTransferArm();
    private final Shooter sh = new Shooter();
    private final Sensors sensors = new Sensors();
    private AutoActions actions;

    private static final boolean INITIAL_SHOT_FAR = false;
    private static final boolean MIDDLE_SHOT_FAR = false;
    private static final boolean FINAL_SHOT_FAR = false;
    private static final double AUTO_WATCHDOG_SECONDS = 30.0;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
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

    public Command autoRoutine() {
        return sequential(
                actions.shoot(INITIAL_SHOT_FAR, "First shot"),
                actions.drive(path1(), "Approach"),
                actions.preparePickup(true),
                actions.drive(path2(), "Straight first pickup"),
                actions.finishPickup(),
                actions.drive(path3(), "Return"),
                actions.shoot(MIDDLE_SHOT_FAR, "Middle shot"),
                actions.drive(path4(), "Approach"),
                actions.preparePickup(true),
                actions.drive(path5(), "Straight end-flower pickup"),
                actions.finishPickup(),
                actions.drive(path6(), "Return"),
                actions.shoot(FINAL_SHOT_FAR, "Final shot"),
                instant(actions::stopMechanisms)
        );
    }

    @Override public void runOpMode() {
        Scheduler.reset();
        try {
            in.initIntakeTransfer(hardwareMap);
            sh.initShooter(hardwareMap);
            sensors.initSensors(hardwareMap);

            follower = Constants.create(hardwareMap);
            if (follower == null) {
                telemetry.update();
                return;
            }
            follower.setPose(startPose);
            follower.update();
            actions = new AutoActions(follower, in, sh, sensors);
            Command routine = autoRoutine();
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            double startTime = System.nanoTime() * 1e-9;
            schedule(routine);
            while (opModeIsActive()) {
                follower.update();
                sh.PIDFShootingLoop();
                actions.updateCollection();
                Scheduler.execute();
                telemetry.addData("Action", actions.status());
                telemetry.addData("X / Y", follower.pose().x() + " / " + follower.pose().y());
                telemetry.addData("Heading degrees", Math.toDegrees(follower.pose().heading()));
                telemetry.addData("Shooter target", sh.curTargetVelocity);
                telemetry.addData("Shooter left / right", sh.getVelocity() + " / " + sh.getVelocity2());
                telemetry.addData("Storage full", sensors.isFull());
                telemetry.update();
                if (actions.failed() || routine.done()) break;

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

    public Path path1() {
        return Paths.curve(startPose, sideControl, sideApproach).heading(AutoActions.linearHeading(startPose, sideApproach));
    }

    public Path path2() {
        return Paths.line(sideApproach, sidePickup).heading(AutoActions.linearHeading(sideApproach, sidePickup));
    }

    public Path path3() {
        return Paths.curve(sidePickup, middleControl, middleShot).heading(AutoActions.linearHeading(sidePickup, middleShot));
    }

    public Path path4() {
        return Paths.curve(middleShot, endControl, endApproach).heading(AutoActions.linearHeading(middleShot, endApproach));
    }

    public Path path5() {
        return Paths.line(endApproach, endPickup).heading(AutoActions.linearHeading(endApproach, endPickup));
    }

    public Path path6() {
        return Paths.curve(endPickup, returnControl, finalShot).heading(AutoActions.linearHeading(endPickup, finalShot));
    }
}
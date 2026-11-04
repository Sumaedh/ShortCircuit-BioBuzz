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
import org.firstinspires.ftc.teamcode.BulkCaching;
import org.firstinspires.ftc.teamcode.IntakeTransferArm;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Shooter;

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

    private static final double CELL_X = 84.1;
    private static final double CELL_Y = 81.7;

    private static double aimAtCell(double x, double y) {
        double deg = Math.toDegrees(Math.atan2(CELL_Y - y, CELL_X - x));
        return deg < 0 ? deg + 360 : deg;
    }

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

    private final Pose startPose = poseFactory.of(84.1, 132.7, aimAtCell(84.1, 132.7));

    private final Pose sideControl1 = poseFactory.of(102.0, 128.0, 0);
    private final Pose sideControl2 = poseFactory.of(110.0, 96.0, 0);
    private final Pose sideApproach = poseFactory.of(116.0, 94.0, 0);
    private final Pose sidePickup = poseFactory.of(130.7, 94.0, 0);

    private final Pose middleControl1 = poseFactory.of(122.0, 94.0, 0);
    private final Pose middleControl2 = poseFactory.of(126.0, 22.0, 0);
    private final Pose middleShot = poseFactory.of(81.45, 23.25, aimAtCell(81.45, 23.25));

    private final Pose endControl1 = poseFactory.of(80.0, 31.0, 0);
    private final Pose endControl2 = poseFactory.of(94.38, 31.0, 0);
    private final Pose endApproach = poseFactory.of(94.38, 24.0, 270);
    private final Pose endPickup = poseFactory.of(94.38, 11.0, 270);

    private final Pose returnControl1 = poseFactory.of(114.0, 22.0, 0);
    private final Pose returnControl2 = poseFactory.of(130.0, 118.0, 0);
    private final Pose finalShot = startPose;

    private final Pose parkPose = poseFactory.of(131, 35.25, 0);
    private final Pose parkControl = poseFactory.of(128, 85, 0);

    public Command autoRoutine() {
        return sequential(
                actions.shoot(INITIAL_SHOT_FAR, "First shot"),
                actions.drive(toSideFlower(), "To side flower"),
                actions.preparePickup(true),
                actions.drive(intakeSideFlower(), "Straight side flower intake"),
                actions.finishPickup(),
                actions.drive(toMiddleShot(), "To middle shot"),
                actions.shoot(MIDDLE_SHOT_FAR, "Middle shot"),
                actions.drive(toEndFlower(), "To end flower"),
                actions.preparePickup(true),
                actions.drive(intakeEndFlower(), "Straight end flower intake"),
                actions.finishPickup(),
                actions.drive(toFinalShot(), "To final shot"),
                actions.shoot(FINAL_SHOT_FAR, "Final shot"),
                actions.park()
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

            actions = new AutoActions(follower, in, sh, sensors, parkPose, parkControl);
            Command routine = autoRoutine();
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            double startTime = System.nanoTime() * 1e-9;
            actions.start(routine);
            while (opModeIsActive()) {
                follower.update();
                sh.PIDFShootingLoop();
                boolean finished = actions.update();
                telemetry.addData("Action", actions.status());
                telemetry.addData("X / Y", follower.pose().x() + " / " + follower.pose().y());
                telemetry.addData("Heading degrees", Math.toDegrees(follower.pose().heading()));
                telemetry.addData("Shooter target", sh.curTargetVelocity);
                telemetry.addData("Shooter left / right", sh.getVelocity() + " / " + sh.getVelocity2());
                telemetry.addData("Storage full", sensors.lastFull());
                telemetry.update();
                if (finished) break;

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
        return Paths.curve(startPose, sideControl1, sideControl2, sideApproach)
                .heading(AutoActions.linearHeading(startPose, sideApproach));
    }

    public Path intakeSideFlower() {
        return Paths.line(sideApproach, sidePickup)
                .heading(AutoActions.linearHeading(sideApproach, sidePickup));
    }

    public Path toMiddleShot() {
        return Paths.curve(sidePickup, middleControl1, middleControl2, middleShot)
                .heading(AutoActions.linearHeading(sidePickup, middleShot));
    }

    public Path toEndFlower() {
        return Paths.curve(middleShot, endControl1, endControl2, endApproach)
                .heading(AutoActions.linearHeading(middleShot, endApproach));
    }

    public Path intakeEndFlower() {
        return Paths.line(endApproach, endPickup)
                .heading(AutoActions.linearHeading(endApproach, endPickup));
    }

    public Path toFinalShot() {
        return Paths.curve(endPickup, returnControl1, returnControl2, finalShot)
                .heading(AutoActions.linearHeading(endPickup, finalShot));
    }
}
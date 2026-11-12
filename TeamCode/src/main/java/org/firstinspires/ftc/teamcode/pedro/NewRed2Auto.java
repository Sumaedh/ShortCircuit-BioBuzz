package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.IntakeTransferArm;
import org.firstinspires.ftc.teamcode.Pitch;
import org.firstinspires.ftc.teamcode.Shooter;

@Autonomous
public class NewRed2Auto extends OpMode {

    // SETUP
    private Follower follower;
    IntakeTransferArm ita = new IntakeTransferArm();
    Shooter shooter = new Shooter();
    Pitch pitch = new Pitch();

    // POSE CREATIONS
    private final PoseFactory poseFactory =
            PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

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

    // PATHINGS BETWEEN POSES
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

    public Command autoRoutine() {
        return sequential(
                // shoot preload
                activateShooter,
                waitForShooter,
                continueShoot,
                transferToShoot,
                pause,
                dectivateShooter,
                follow(follower, toSideFlower()),
                armDown,
                intakeOn,
                follow(follower, intakeSideFlower()),
                pause,
                stopIntake,
                followAndPrepArm(toFarShot()),
                waitForShooter,
                continueShoot,
                transferToShoot,
                pause,
                dectivateShooter,
                follow(follower, toEndFlower()),
                follow(follower, intakeEndFlower()),
                armDown,
                intakeOn,
                follow(follower, backOutOfEndFlower()),
                stopIntake,
                armUp,
                activateShooter,
                follow(follower, backToFarShot()),
                waitForShooter,
                continueShoot,
                transferToShoot,
                pause,
                dectivateShooter,
                follow(follower, toGarden()),
                follow(follower, toPark())
        );
    }

    // FOR FLOWERS
    public Command followAndPrepArm(Path path) {
        return parallel(
                follow(follower, path),
                sequential(
                        instant(() -> shooter.setCurTargetVelocity("off")),
                        waitMs(1000), // 12 inches from end
                        instant(() -> ita.armUp())
                )
        );
    }

    // GENERAL
    Command pause = waitMs(2000);

    // SHOOTER
    Command activateShooter =
            instant(() -> shooter.setCurTargetVelocity("on"));
    Command dectivateShooter =
            instant(() -> shooter.setCurTargetVelocity("off"));
    Command waitForShooter =
            waitUntil(() -> shooter.shooterAtTarget());

    // STOPPER
    Command stopShoot = instant(() -> ita.stopShoot());
    Command continueShoot = instant(() -> ita.continueShoot());

    // TRANSFER
    Command transferToShoot = instant(() -> ita.transferShoot());
    Command stopIntake = instant(() -> ita.stopShoot());
    Command intakeOn = instant(() -> ita.intakeOn());

    // ARM
    Command armUp = instant(() -> ita.armUp());
    Command armDown = instant(() -> ita.intakeFlower());

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        ita.initIntakeTransfer(hardwareMap);
        shooter.initShooter(hardwareMap);
        pitch.initPitch(hardwareMap);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        shooter.PIDFShootingLoop();
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
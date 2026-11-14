package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.IntakeTransferArm;
import org.firstinspires.ftc.teamcode.Pitch;
import org.firstinspires.ftc.teamcode.Shooter;

//@Autonomous
public abstract class Meet1Red2Auto extends OpMode {

    private Follower follower;

    private final IntakeTransferArm ita = new IntakeTransferArm();
    private final Shooter shooter = new Shooter();
    private final Pitch pitch = new Pitch();

    private Pose startPose;

    protected abstract PoseFactory getPoseFactory();

    @Override
    public void init() {
        Scheduler.reset();

        PoseFactory poseFactory = getPoseFactory();
        poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

        startPose = poseFactory.of(84.1, 129.5, 270);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        ita.initIntakeTransfer(hardwareMap);
        shooter.initShooter(hardwareMap);
        pitch.initPitch(hardwareMap);
    }

    private Command shootAtStart() {
        return sequential(
                instant(() -> shooter.setCurTargetVelocity("on")),

                waitUntil(() -> shooter.shooterAtTarget()),

                instant(() -> ita.continueShoot()),
                instant(() -> ita.transferShoot()),

                waitMs(2000),

                instant(() -> shooter.setCurTargetVelocity("off"))
        );
    }

    @Override
    public void start() {
        schedule(shootAtStart());
    }

    @Override
    public void loop() {
        follower.update();

        Scheduler.execute();

        shooter.PIDFShootingLoop();

        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Start Pose", startPose);
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntakeTransferArm;
import org.firstinspires.ftc.teamcode.Pitch;
import org.firstinspires.ftc.teamcode.Shooter;

@Autonomous(name = "Meet 1 Red 1", group = "Meet1")
public class Meet1Red1 extends OpMode {
    private static final double SHOOTER_TIMEOUT_MS = 3000;
    private static final double FEED_MS = 2000;

    private enum State { SPIN_UP, FEED, DONE }

    private final IntakeTransferArm ita = new IntakeTransferArm();
    private final Shooter shooter = new Shooter();
    private final Pitch pitch = new Pitch();
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.SPIN_UP;

    @Override
    public void init() {
        ita.initIntakeTransfer(hardwareMap);
        shooter.initShooter(hardwareMap);
        pitch.initPitch(hardwareMap);
    }

    @Override
    public void start() {
        shooter.setCurTargetVelocity("on");
        timer.reset();
        state = State.SPIN_UP;
    }

    @Override
    public void loop() {
        switch (state) {
            case SPIN_UP:
                if (shooter.shooterAtTarget() || timer.milliseconds() > SHOOTER_TIMEOUT_MS) {
                    ita.continueShoot();
                    ita.transferShoot();
                    timer.reset();
                    state = State.FEED;
                }
                break;

            case FEED:
                if (timer.milliseconds() > FEED_MS) {
                    ita.stopShoot();
                    shooter.setCurTargetVelocity("off");
                    state = State.DONE;
                    // fijro
                }
                break;

            case DONE:
                break;
        }

        shooter.PIDFShootingLoop();

        telemetry.addData("State", state);
        telemetry.update();
    }
}
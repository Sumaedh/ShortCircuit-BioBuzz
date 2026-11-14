package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntakeTransferArm;
import org.firstinspires.ftc.teamcode.Pitch;
import org.firstinspires.ftc.teamcode.Shooter;


public abstract class Meet1Autobase extends OpMode {

    private static final String FRONT_LEFT_NAME  = "frontLeft";
    private static final String FRONT_RIGHT_NAME = "frontRight";
    private static final String BACK_LEFT_NAME   = "backLeft";
    private static final String BACK_RIGHT_NAME  = "backRight";

    private static final double TICKS_PER_REV     = 336;
    private static final double WHEEL_DIAMETER_IN = 3.78;
    private static final double TICKS_PER_INCH    = TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI);

    private static final double STRAFE_MULTIPLIER = 1.1;

    private static final double PARK_POWER = 0.4;

    private static final double SHOOTER_TIMEOUT_MS  = 3000;
    private static final double FEED_MS = 2000;
    private static final double PARK_LEG_TIMEOUT_MS = 4000;

    private enum State { SPIN_UP, FEED, PARK_FORWARD, PARK_STRAFE, DONE }

    private final IntakeTransferArm ita = new IntakeTransferArm();
    private final Shooter shooter = new Shooter();
    private final Pitch pitch = new Pitch();
    private final ElapsedTime timer = new ElapsedTime();

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private State state = State.SPIN_UP;

    protected abstract double parkForwardInches();

    protected abstract double parkStrafeInches();

    @Override
    public void init() {
        ita.initIntakeTransfer(hardwareMap);
        shooter.initShooter(hardwareMap);
        pitch.initPitch(hardwareMap);

        frontLeft  = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft   = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight  = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : driveMotors()) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
                    startLeg(parkForwardInches(), 0);
                    state = State.PARK_FORWARD;
                }
                break;

            case PARK_FORWARD:
                if (legDone()) {
                    startLeg(0, parkStrafeInches());
                    state = State.PARK_STRAFE;
                }
                break;

            case PARK_STRAFE:
                if (legDone()) {
                    stopDrive();
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }

        shooter.PIDFShootingLoop();

        telemetry.addData("State", state);
        telemetry.addData("FL / FR", "%d / %d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.addData("BL / BR", "%d / %d",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
    }

    private DcMotor[] driveMotors() {
        return new DcMotor[] { frontLeft, frontRight, backLeft, backRight };
    }

    private void startLeg(double forwardInches, double strafeInches) {
        int forward = (int) Math.round(forwardInches * TICKS_PER_INCH);
        int strafe  = (int) Math.round(strafeInches * TICKS_PER_INCH * STRAFE_MULTIPLIER);

        setTarget(frontLeft,  forward + strafe);
        setTarget(frontRight, forward - strafe);
        setTarget(backLeft,   forward - strafe);
        setTarget(backRight,  forward + strafe);

        for (DcMotor m : driveMotors()) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(PARK_POWER);
        }
        timer.reset();
    }

    private void setTarget(DcMotor motor, int deltaTicks) {
        motor.setTargetPosition(motor.getCurrentPosition() + deltaTicks);
    }

    private boolean legDone() {
        if (timer.milliseconds() > PARK_LEG_TIMEOUT_MS) {
            return true;
        }
        for (DcMotor m : driveMotors()) {
            if (m.isBusy()) {
                return false;
            }
        }
        return true;
    }

    private void stopDrive() {
        if (frontLeft == null) {
            return;
        }
        for (DcMotor m : driveMotors()) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
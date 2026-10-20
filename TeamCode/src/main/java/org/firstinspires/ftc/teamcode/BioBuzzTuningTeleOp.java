package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BIOBUZZ Tuning & TeleOp", group = "BIOBUZZ")
public class BioBuzzTuningTeleOp extends OpMode {


    public static double INTAKE_POWER = 1.0;
    public static double FEED_POWER = 1.0;

    public static double F_SHOOTING = 21.5;
    public static double P_SHOOTING = 500.0;
    public static double LONG_VELOCITY = 1470.0;
    public static double SHORT_VELOCITY = 1200.0;
    public static double SHOOTER_TOLERANCE = 50.0;

    public static double SHOT_STOPPER_BLOCKED = 0.0;
    public static double SHOT_STOPPER_OPEN = 0.5; // Tune this

    public static double INTAKE_STOPPER_BLOCKED = 0.0;
    public static double INTAKE_STOPPER_OPEN = 0.5; // Tune this

    public static double ARM_UP_POSITION = 0.0;
    public static double ARM_DOWN_POSITION = 0.4; // Tune this
    public static double ARM_BLOCK_POSITION = 0.2; // Tune this for blocking flower balls

    public static boolean SENSOR_1_ACTIVE_HIGH = true;
    public static boolean SENSOR_2_ACTIVE_HIGH = true;
    public static boolean FULL_REQUIRES_BOTH = false;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeLeft, intakeRight;
    private DcMotorEx shootingMotor, shootingMotor2;
    private Servo shotStopper, intakeStopper, flowerArm;
    private DigitalChannel fullSensor1, fullSensor2;

    public double curTargetVelocity = 0;

    enum DriveState { NORMAL, SLOW }
    enum IntakeState { OFF, ON, REVERSE, FULL, FEED }
    enum ShooterState { OFF, SHORT, LONG }
    enum StopperState { BLOCKED, OPEN }
    enum ArmState { UP, DOWN, BLOCK }

    DriveState driveState = DriveState.NORMAL;
    IntakeState intakeState = IntakeState.OFF;
    ShooterState shooterState = ShooterState.OFF;
    StopperState shotState = StopperState.BLOCKED;
    StopperState intakeStopperState = StopperState.BLOCKED;
    ArmState armState = ArmState.UP;

    private boolean previousA, previousY;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[]{intakeLeft, intakeRight}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        shootingMotor = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shootingMotor2 = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shootingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootingMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[]{shootingMotor, shootingMotor2}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P_SHOOTING, 0, 0, F_SHOOTING));
            motor.setVelocity(0);
        }
        curTargetVelocity = 0;

        shotStopper = hardwareMap.get(Servo.class, "shotStopper");
        intakeStopper = hardwareMap.get(Servo.class, "intakeStopper");
        flowerArm = hardwareMap.get(Servo.class, "flowerArm");

        shotStopper.setDirection(Servo.Direction.FORWARD);
        intakeStopper.setDirection(Servo.Direction.FORWARD);
        flowerArm.setDirection(Servo.Direction.FORWARD);

        shotStopper.setPosition(SHOT_STOPPER_BLOCKED);
        intakeStopper.setPosition(INTAKE_STOPPER_BLOCKED);
        flowerArm.setPosition(ARM_UP_POSITION);

        fullSensor1 = hardwareMap.get(DigitalChannel.class, "fullSensor1");
        fullSensor2 = hardwareMap.get(DigitalChannel.class, "fullSensor2");
        fullSensor1.setMode(DigitalChannel.Mode.INPUT);
        fullSensor2.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("BIOBUZZ Tuning & TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a && !previousA;
        boolean yPressed = gamepad1.y && !previousY;
        previousA = gamepad1.a;
        previousY = gamepad1.y;

        boolean s1 = fullSensor1.getState() == SENSOR_1_ACTIVE_HIGH;
        boolean s2 = fullSensor2.getState() == SENSOR_2_ACTIVE_HIGH;
        boolean full = FULL_REQUIRES_BOTH ? (s1 && s2) : (s1 || s2);

        updateDriveState();
        updateShooterState();
        updateIntakeState(aPressed, full);
        updateArmState(yPressed);

        executeDriveStateMachine();
        executeShooterStateMachine();
        executeIntakeAndStopperStateMachine();
        executeArmStateMachine();

        telemetry.addData("Drive State", driveState);
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shot Stopper State", shotState);
        telemetry.addData("Flower Arm State", armState);
        telemetry.addData("Storage Full", full);
        telemetry.addData("Shooter Target Vel", curTargetVelocity);
        telemetry.addData("Shooter Left Vel", shootingMotor.getVelocity());
        telemetry.addData("Shooter Right Vel", shootingMotor2.getVelocity());
        telemetry.update();
    }


    private void updateDriveState() {
        switch (driveState) {
            case NORMAL:
                if (gamepad1.left_trigger > 0.75) driveState = DriveState.SLOW;
                break;
            case SLOW:
                if (gamepad1.left_trigger <= 0.75) driveState = DriveState.NORMAL;
                break;
        }
    }

    private void updateShooterState() {
        switch (shooterState) {
            case OFF:
                if (gamepad1.right_trigger > 0.75) shooterState = ShooterState.LONG;
                else if (gamepad1.left_bumper) shooterState = ShooterState.SHORT;
                break;
            case SHORT:
                if (gamepad1.back || !gamepad1.left_bumper) shooterState = ShooterState.OFF;
                else if (gamepad1.right_trigger > 0.75) shooterState = ShooterState.LONG;
                break;
            case LONG:
                if (gamepad1.back || gamepad1.right_trigger <= 0.75) shooterState = ShooterState.OFF;
                else if (gamepad1.left_bumper) shooterState = ShooterState.SHORT;
                break;
        }
    }

    private void updateIntakeState(boolean aPressed, boolean full) {
        switch (intakeState) {
            case OFF:
                if (gamepad1.x) intakeState = IntakeState.REVERSE;
                else if (gamepad1.dpad_up) intakeState = IntakeState.FEED;
                else if (aPressed) intakeState = full ? IntakeState.FULL : IntakeState.ON;
                break;
            case ON:
                if (gamepad1.back || gamepad1.b) intakeState = IntakeState.OFF;
                else if (gamepad1.x) intakeState = IntakeState.REVERSE;
                else if (gamepad1.dpad_up) intakeState = IntakeState.FEED;
                else if (full) intakeState = IntakeState.FULL;
                break;
            case REVERSE:
                if (gamepad1.back || gamepad1.b || !gamepad1.x) intakeState = IntakeState.OFF;
                break;
            case FULL:
                if (gamepad1.back || gamepad1.b) intakeState = IntakeState.OFF;
                else if (gamepad1.x) intakeState = IntakeState.REVERSE;
                else if (gamepad1.dpad_up) intakeState = IntakeState.FEED;
                break;
            case FEED:
                if (gamepad1.back || gamepad1.b || !gamepad1.dpad_up) intakeState = IntakeState.OFF;
                break;
        }
    }

    private void updateArmState(boolean yPressed) {
        switch (armState) {
            case UP:
                if (gamepad1.dpad_left) armState = ArmState.BLOCK;
                else if (yPressed) armState = ArmState.DOWN;
                break;
            case DOWN:
                if (gamepad1.back || gamepad1.dpad_down) armState = ArmState.UP;
                else if (gamepad1.dpad_left) armState = ArmState.BLOCK;
                else if (yPressed) armState = ArmState.UP;
                break;
            case BLOCK:
                if (gamepad1.back || gamepad1.dpad_down) armState = ArmState.UP;
                else if (yPressed) armState = ArmState.DOWN;
                break;
        }
    }

    private void executeDriveStateMachine() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double scale = (driveState == DriveState.SLOW) ? (1.0 / 3.0) : 1.0;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        frontLeft.setPower((-y + x + rx) / denominator * scale);
        frontRight.setPower((-y - x - rx) / denominator * scale);
        backLeft.setPower((-y - x + rx) / denominator * scale);
        backRight.setPower((-y + x - rx) / denominator * scale);
    }

    private void executeShooterStateMachine() {
        switch (shooterState) {
            case OFF: curTargetVelocity = 0; break;
            case SHORT: curTargetVelocity = SHORT_VELOCITY; break;
            case LONG: curTargetVelocity = LONG_VELOCITY; break;
        }

        for (DcMotorEx motor : new DcMotorEx[]{shootingMotor, shootingMotor2}) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P_SHOOTING, 0, 0, F_SHOOTING));
            motor.setVelocity(curTargetVelocity);
        }
    }

    private void executeIntakeAndStopperStateMachine() {
        shotState = StopperState.BLOCKED;
        intakeStopperState = StopperState.BLOCKED;

        switch (intakeState) {
            case OFF:
            case FULL:
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                break;
            case ON:
                intakeLeft.setPower(INTAKE_POWER);
                intakeRight.setPower(INTAKE_POWER);
                intakeStopperState = StopperState.OPEN;
                break;
            case REVERSE:
                intakeLeft.setPower(-INTAKE_POWER);
                intakeRight.setPower(-INTAKE_POWER);
                intakeStopperState = StopperState.OPEN;
                break;
            case FEED:
                boolean atTarget = (curTargetVelocity > 0)
                        && Math.abs(shootingMotor.getVelocity() - curTargetVelocity) <= SHOOTER_TOLERANCE
                        && Math.abs(shootingMotor2.getVelocity() - curTargetVelocity) <= SHOOTER_TOLERANCE;
                if (atTarget) {
                    intakeLeft.setPower(FEED_POWER);
                    intakeRight.setPower(FEED_POWER);
                    shotState = StopperState.OPEN;
                } else {
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                }
                break;
        }

        shotStopper.setPosition(shotState == StopperState.OPEN ? SHOT_STOPPER_OPEN : SHOT_STOPPER_BLOCKED);
        intakeStopper.setPosition(intakeStopperState == StopperState.OPEN ? INTAKE_STOPPER_OPEN : INTAKE_STOPPER_BLOCKED);
    }

    private void executeArmStateMachine() {
        switch (armState) {
            case UP:
                flowerArm.setPosition(ARM_UP_POSITION);
                break;
            case DOWN:
                flowerArm.setPosition(ARM_DOWN_POSITION);
                break;
            case BLOCK:
                flowerArm.setPosition(ARM_BLOCK_POSITION);
                break;
        }
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        shootingMotor.setVelocity(0);
        shootingMotor2.setVelocity(0);
        shotStopper.setPosition(SHOT_STOPPER_BLOCKED);
        intakeStopper.setPosition(INTAKE_STOPPER_BLOCKED);
        flowerArm.setPosition(ARM_UP_POSITION);
    }
}
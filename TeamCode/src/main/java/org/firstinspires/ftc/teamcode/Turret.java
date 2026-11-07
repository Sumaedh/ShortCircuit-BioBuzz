package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Turret Test", group = "BIO-BUZZ")
public class Turret extends OpMode {
    private DcMotorEx turretMotor;

    GoBildaPinpointDriver pinpointDriver;

    private final double ticksPerRevolution = 537.7;
    private final double gearRatio = 1.0;

    private final double minAngle = -3.0 * Math.PI / 4.0;
    private final double maxAngle = 3.0 * Math.PI / 4.0;
    private final double zeroOffsetAngle = 0.0;

    private final double kP = 1.146;
    private final double kI = 0.0573;
    private final double kD = 0.0458;
    private final double kF = 0.02;
    private final double integralLimit = 5.0;
    private final double deadband = Math.PI / 180.0;
    private final double maxPower = 0.5;

    private final double podOffsetX = 0.0;
    private final double podOffsetY = 0.0;

    private final double startX = 84.1051;
    private final double startY = 132.7;
    private final double startHeading = 3.0 * Math.PI / 2.0;

    private final double targetXleft = 78.45;
    private final double targetYleft = 59.55;
    private final double targetXright = 88.55;
    private final double targetYright = 76.25;

    private final double fieldCenterY = 70.75;
    private final double sideSwitchBand = 2.0;

    private boolean targetIsRight = true;
    private double previousAngle = 0.0;
    private double previousError = 0.0;
    private double integral = 0.0;
    private double previousTimestamp = -1.0;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        BulkCaching.enableAuto(hardwareMap);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(podOffsetX, podOffsetY, DistanceUnit.INCH);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();
    }

    @Override
    public void start() {
        pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.RADIANS, startHeading));
        timer.reset();
    }

    @Override
    public void loop() {
        pinpointDriver.update();
        Pose2D pose = pinpointDriver.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double robotHeading = pose.getHeading(AngleUnit.RADIANS);

        if (robotY > fieldCenterY + sideSwitchBand) {
            targetIsRight = true;
        } else if (robotY < fieldCenterY - sideSwitchBand) {
            targetIsRight = false;
        }

        double targetX;
        double targetY;
        if (targetIsRight) {
            targetX = targetXright;
            targetY = targetYright;
        } else {
            targetX = targetXleft;
            targetY = targetYleft;
        }

        double offsetX = targetX - robotX;
        double offsetY = targetY - robotY;

        double angle = Math.atan2(offsetY, offsetX);
        double adjustedAngle = AngleUnit.normalizeRadians(angle - robotHeading);
        double targetAngle = Range.clip(adjustedAngle, minAngle, maxAngle);

        double motorRotations = turretMotor.getCurrentPosition() / ticksPerRevolution;
        double currentAngle = (motorRotations / gearRatio) * 2.0 * Math.PI + zeroOffsetAngle;

        double error = targetAngle - currentAngle;

        double now = timer.seconds();
        double dt = previousTimestamp >= 0.0 ? now - previousTimestamp : 0.0;
        previousTimestamp = now;

        if (error * previousError < 0.0) {
            integral = 0.0;
        }
        integral = Range.clip(integral + error * dt, -integralLimit, integralLimit);
        double derivative = dt > 0.0 ? -(currentAngle - previousAngle) / dt : 0.0;

        double output = kP * error + kI * integral + kD * derivative + kF * Math.signum(error);

        if (Math.abs(error) < deadband) {
            integral = 0.0;
            output = 0.0;
        }

        if ((currentAngle >= maxAngle && output > 0.0) || (currentAngle <= minAngle && output < 0.0)) {
            integral = 0.0;
            output = 0.0;
        }

        output = Range.clip(output, -maxPower, maxPower);
        turretMotor.setPower(output);

        previousAngle = currentAngle;
        previousError = error;

        telemetry.addData("Robot X / Y", "%.1f / %.1f", robotX, robotY);
        telemetry.addData("Robot heading (rad)", "%.3f", robotHeading);
        telemetry.addData("Target CELL", targetIsRight ? "RIGHT (far)" : "LEFT (audience)");
        telemetry.addData("Wanted angle (rad)", "%.3f", adjustedAngle);
        telemetry.addData("Target angle (rad)", "%.3f", targetAngle);
        telemetry.addData("Turret angle (rad)", "%.3f", currentAngle);
        telemetry.addData("Power", "%.2f", output);
        telemetry.update();
    }

    @Override
    public void stop() {
        turretMotor.setPower(0.0);
    }
}

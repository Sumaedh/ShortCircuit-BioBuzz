package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret Dashboard Calibrator", group = "BIOBUZZ")
public class TurretDashboardCalibrator extends LinearOpMode {

    private TurretServoCalibration turret;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() {
        turret = new TurretServoCalibration(hardwareMap, "turretServo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Use Dashboard to tune values.");
        telemetry.addLine("A/B adjust target angle by STEP_DEGREES");
        telemetry.addLine("X = center zero");
        telemetry.addLine("Y = set target to 0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !prevA) {
                TurretServoConfig.TARGET_DEGREES -= TurretServoConfig.STEP_DEGREES;
            }
            if (gamepad1.b && !prevB) {
                TurretServoConfig.TARGET_DEGREES += TurretServoConfig.STEP_DEGREES;
            }
            if (gamepad1.x && !prevX) {
                TurretServoConfig.TARGET_DEGREES = 0.0;
                turret.setToZero();
            }
            if (gamepad1.y && !prevY) {
                TurretServoConfig.TARGET_DEGREES = 0.0;
            }

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevY = gamepad1.y;


            turret.setTargetDegrees(TurretServoConfig.TARGET_DEGREES);

            telemetry.addData("ZERO_POSITION", TurretServoConfig.ZERO_POSITION);
            telemetry.addData("LEFT_LIMIT_POS", TurretServoConfig.LEFT_LIMIT_POS);
            telemetry.addData("RIGHT_LIMIT_POS", TurretServoConfig.RIGHT_LIMIT_POS);
            telemetry.addData("TARGET_DEGREES", TurretServoConfig.TARGET_DEGREES);
            telemetry.addData("Left Degrees", TurretServoConfig.LEFT_LIMIT_DEGREES);
            telemetry.addData("Right Degrees", TurretServoConfig.RIGHT_LIMIT_DEGREES);
            telemetry.update();
        }
    }
}
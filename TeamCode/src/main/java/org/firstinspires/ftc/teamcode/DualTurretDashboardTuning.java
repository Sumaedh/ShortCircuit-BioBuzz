package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Dual Turret Dashboard Tuning", group = "BIOBUZZ")
public class DualTurretDashboardTuning extends LinearOpMode {

    private DualTurretServoController turret;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;

    @Override
    public void runOpMode() {
        turret = new DualTurretServoController(hardwareMap, "leftTurretServo", "rightTurretServo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !prevA) {
                DualTurretServoConfig.TARGET_DEGREES -= DualTurretServoConfig.STEP_DEGREES;
            }
            if (gamepad1.b && !prevB) {
                DualTurretServoConfig.TARGET_DEGREES += DualTurretServoConfig.STEP_DEGREES;
            }
            if (gamepad1.x && !prevX) {
                DualTurretServoConfig.TARGET_DEGREES = 0.0;
            }

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;

            turret.setTargetDegrees(DualTurretServoConfig.TARGET_DEGREES);

            telemetry.addData("ZERO_POSITION", DualTurretServoConfig.ZERO_POSITION);
            telemetry.addData("LEFT_LIMIT_POS", DualTurretServoConfig.LEFT_LIMIT_POS);
            telemetry.addData("RIGHT_LIMIT_POS", DualTurretServoConfig.RIGHT_LIMIT_POS);
            telemetry.addData("TARGET_DEGREES", DualTurretServoConfig.TARGET_DEGREES);
            telemetry.update();
        }
    }
}
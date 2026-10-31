package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret Dashboard Tuning", group = "BIOBUZZ")
public class TurretDashboardTuning extends LinearOpMode {

    private TurretServoZero turret;

    @Override
    public void runOpMode() {
        turret = new TurretServoZero(hardwareMap, "turretServo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            turret.setTurretAngleDegrees(TurretServoConfig.TURRET_TARGET_DEGREES);

            telemetry.addData("Zero Position", TurretServoConfig.ZERO_POSITION);
            telemetry.addData("Target Degrees", TurretServoConfig.TURRET_TARGET_DEGREES);
            telemetry.addData("Total Range", TurretServoConfig.TOTAL_RANGE_DEGREES);
            telemetry.update();
        }
    }
}
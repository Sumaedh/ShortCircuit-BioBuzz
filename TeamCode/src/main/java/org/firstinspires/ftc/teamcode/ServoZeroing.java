package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ServoZeroingZeroing", group = "BIOBUZZ")
public class ServoZeroing extends LinearOpMode {


    public static double TURRET_TARGET_POSITION = 0.5;

    @Override
    public void runOpMode() {
        Servo turretServo1 = hardwareMap.get(Servo.class, "turret_servo1");
        Servo turretServo2 = hardwareMap.get(Servo.class, "turret_servo2");
        Servo turretServo3 = hardwareMap.get(Servo.class, "turret_servo3");
        Servo turretServo4 = hardwareMap.get(Servo.class, "turret_servo4");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Ready. Connect to http://192.168.43");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            turretServo1.setPosition(TURRET_TARGET_POSITION);
            turretServo2.setPosition(TURRET_TARGET_POSITION);
            turretServo3.setPosition(TURRET_TARGET_POSITION);
            turretServo4.setPosition(TURRET_TARGET_POSITION);

            telemetry.addData("Current Target Position", TURRET_TARGET_POSITION);
            telemetry.update();
        }
    }
}
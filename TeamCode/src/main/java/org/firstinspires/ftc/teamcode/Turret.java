package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret extends OpMode {
    private DcMotorEx turretMotor;


    // FOR BLUE
    GoBildaPinpointDriver pinpointDriver;

    private final double ticksPerRevolution = 537.7;

    private final double gearRatio = 1.0;

    private final double minAngleAngle = -135.0;
    private final double maxAngleAngle = 135.0;

    private double targetAngle = 0.0;
    private double previousError = 0.0;
    private double integral = 0.0;
    private double previousTimestamp = -1.0;

    private double zeroOffsetAngles = 0.0;

    int targetXleft = 0;
    int targetYleft = 0;
    int targetXright = 0;
    int targetYright = 0;


    int robotX = 0;
    int robotY = 0;
    int robotHeading = 0;


    @Override
    public void init() {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    @Override
    public void loop() {


        //Pose2D startPos = Pose2D(DistanceUnit unit, double x, double y, AngleUnit, double heading)
        // TODO 1: GET POSES FROM PINPOINT

        int targetX;
        int targetY;

        // TODO 2: DETERMINE IF OUR TARGET IS ON THE LEFT OR RIGHT SIDE AND SET THAT AS TARGET
        if (true) {
            targetX = targetXleft;
            targetY = targetYleft;
        } else {
            targetX = targetXright;
            targetY = targetYright;
        }

        int offsetX = targetX - robotX;
        int offsetY = targetY - robotY;

        double angle = Math.atan2(offsetY, offsetX);

        //double adjustedAngle =


    }
}

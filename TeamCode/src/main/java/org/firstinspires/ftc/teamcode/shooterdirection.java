package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp
public class shooterdirection extends OpMode {


    DcMotorEx leftShootingMotor, rightShootingMotor;

    public static double left = 0;
    public static double right = 0;

    @Override
    public void init() {
        leftShootingMotor = hardwareMap.get(DcMotorEx.class, "leftShootingMotor");
        rightShootingMotor = hardwareMap.get(DcMotorEx.class, "rightShootingMotor");

        leftShootingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShootingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {

        leftShootingMotor.setPower(left);
        rightShootingMotor.setPower(right);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BioBuzzIntake {
    DcMotor intakeLeft, intakeRight;
    public double INTAKE_POWER = 1.0;
    public double FEED_POWER = 1.0;

    public void initIntake(HardwareMap hwMap) {
        intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hwMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[]{intakeLeft, intakeRight}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void intakeOn() { setPower(INTAKE_POWER); }
    public void intakeOff() { setPower(0); }
    public void intakeReverse() { setPower(-INTAKE_POWER); }
    public void feedShooter() { setPower(FEED_POWER); }

    public void setPower(double power) {
        if (intakeLeft != null) intakeLeft.setPower(power);
        if (intakeRight != null) intakeRight.setPower(power);
    }

}
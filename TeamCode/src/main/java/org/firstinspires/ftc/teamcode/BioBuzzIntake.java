package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class BioBuzzIntake {
    private DcMotor left, right;
    private final boolean dryRun = BioBuzzConfig.DRY_RUN;
    private double commandedPower;
    public void init(HardwareMap map) {
        if (dryRun) return;
        left = map.get(DcMotor.class, BioBuzzConfig.INTAKE_LEFT);
        right = map.get(DcMotor.class, BioBuzzConfig.INTAKE_RIGHT);
        left.setDirection(BioBuzzConfig.INTAKE_LEFT_DIRECTION);
        right.setDirection(BioBuzzConfig.INTAKE_RIGHT_DIRECTION);
        for (DcMotor motor : new DcMotor[]{left, right}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void intakeOn() { setPower(BioBuzzConfig.INTAKE_POWER); }
    public void intakeOff() { setPower(0); }
    public void intakeReverse() { setPower(-BioBuzzConfig.INTAKE_POWER); }
    public void feedShooter() { setPower(BioBuzzConfig.TRANSFER_FEED_POWER); }
    public void setPower(double power) {
        if (!Double.isFinite(power) || Math.abs(power) > 1) throw new IllegalArgumentException("Intake power must be in [-1,1]");
        commandedPower = power;
        if (!dryRun) {
            if (left != null) left.setPower(power);
            if (right != null) right.setPower(power);
        }
    }
    public double getPower() { return commandedPower; }
    public Runnable intakeOnParametric() { intakeOn(); return null; }
    public Runnable intakeOffParametric() { intakeOff(); return null; }
    public Runnable intakeReverseParametric() { intakeReverse(); return null; }
    public Runnable feedShooterParametric() { feedShooter(); return null; }
    public Runnable setPowerParametric(double power) { setPower(power); return null; }
}
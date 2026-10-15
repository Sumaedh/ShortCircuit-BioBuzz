package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/** Two encoder-controlled shooter motors; feeding requires BOTH to stay at target speed. */
public final class BioBuzzShooter {
    private DcMotorEx left, right;
    private final boolean dryRun = BioBuzzConfig.DRY_RUN;
    private double target, stableSince = Double.NaN, targetChangedAt;
    private double leftVelocity, rightVelocity;
    private boolean ready;

    public void init(HardwareMap map) {
        if (dryRun) return;
        left = map.get(DcMotorEx.class, BioBuzzConfig.SHOOTER_LEFT);
        right = map.get(DcMotorEx.class, BioBuzzConfig.SHOOTER_RIGHT);
        left.setDirection(BioBuzzConfig.SHOOTER_LEFT_DIRECTION);
        right.setDirection(BioBuzzConfig.SHOOTER_RIGHT_DIRECTION);
        for (DcMotorEx motor : new DcMotorEx[]{left, right}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (BioBuzzConfig.USE_CUSTOM_SHOOTER_PIDF) {
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(BioBuzzConfig.SHOOTER_P, BioBuzzConfig.SHOOTER_I,
                                BioBuzzConfig.SHOOTER_D, BioBuzzConfig.SHOOTER_F));
            }
        }
    }
    public void shooterClose() {
        setVelocity(dryRun && BioBuzzConfig.CLOSE_TPS == 0 ? 1000 : BioBuzzConfig.CLOSE_TPS);
    }
    public void shooterFar() {
        setVelocity(dryRun && BioBuzzConfig.FAR_TPS == 0 ? 1500 : BioBuzzConfig.FAR_TPS);
    }
    public void shooterOff() { setVelocity(0); }
    public void setVelocity(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond < 0) throw new IllegalArgumentException("Shooter target must be finite and >= 0");
        if (target != ticksPerSecond) {
            target = ticksPerSecond;
            targetChangedAt = BioBuzzConfig.now();
            stableSince = Double.NaN;
            ready = false;
        }
        if (ticksPerSecond == 0) { ready = false; stableSince = Double.NaN; }
        if (!dryRun) {
            for (DcMotorEx motor : new DcMotorEx[]{left, right}) {
                if (motor == null) continue;
                if (target == 0) { motor.setVelocity(0); motor.setPower(0); }
                else motor.setVelocity(target);
            }
        }
    }
    public void update(double now) {
        double simulated = now - targetChangedAt >= 0.5 ? target : 0;
        leftVelocity = dryRun ? simulated : left.getVelocity();
        rightVelocity = dryRun ? simulated : right.getVelocity();
        boolean inBand = target > 0
                && Math.abs(leftVelocity - target) <= BioBuzzConfig.SHOOTER_TOLERANCE_TPS
                && Math.abs(rightVelocity - target) <= BioBuzzConfig.SHOOTER_TOLERANCE_TPS;
        if (!inBand) { stableSince = Double.NaN; ready = false; return; }
        if (Double.isNaN(stableSince)) stableSince = now;
        ready = now - stableSince >= BioBuzzConfig.SHOOTER_READY_SECONDS;
    }
    public boolean isReady() { return ready; }
    public double getTargetVelocity() { return target; }
    public double getLeftVelocity() { return leftVelocity; }
    public double getRightVelocity() { return rightVelocity; }
    public Runnable shooterCloseParametric() { shooterClose(); return null; }
    public Runnable shooterFarParametric() { shooterFar(); return null; }
    public Runnable shooterOffParametric() { shooterOff(); return null; }
    public Runnable setVelocityParametric(double tps) { setVelocity(tps); return null; }
}
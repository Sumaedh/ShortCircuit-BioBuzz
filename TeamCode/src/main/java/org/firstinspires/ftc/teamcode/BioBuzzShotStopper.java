package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Positional servo: commands a position, not continuous-rotation power. */
public final class BioBuzzShotStopper {
    private Servo servo;
    private final boolean dryRun = BioBuzzConfig.DRY_RUN;
    private double commandedPosition;
    public void init(HardwareMap map) {
        if (!dryRun) {
            servo = map.get(Servo.class, BioBuzzConfig.SHOT_STOPPER);
            servo.setDirection(BioBuzzConfig.SHOT_STOPPER_DIRECTION);
        }
        block();
    }
    public void block() { setPosition(BioBuzzConfig.SHOT_BLOCKED); }
    public void open() { setPosition(BioBuzzConfig.SHOT_OPEN); }
    public void setPosition(double position) {
        if (!Double.isFinite(position) || position < 0 || position > 1) {
            throw new IllegalArgumentException("Servo position must be in [0,1]");
        }
        commandedPosition = position;
        if (!dryRun && servo != null) servo.setPosition(position);
    }
    public double getCommandedPosition() { return commandedPosition; }
    public Runnable blockParametric() { block(); return null; }
    public Runnable openParametric() { open(); return null; }
    public Runnable setPositionParametric(double p) { setPosition(p); return null; }
}
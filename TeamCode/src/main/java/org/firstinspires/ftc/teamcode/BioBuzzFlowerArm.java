package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Positional servo: commands a position, not continuous-rotation power. */
public final class BioBuzzFlowerArm {
    private Servo servo;
    private final boolean dryRun = BioBuzzConfig.DRY_RUN;
    private double commandedPosition;
    public void init(HardwareMap map) {
        if (!dryRun) {
            servo = map.get(Servo.class, BioBuzzConfig.FLOWER_ARM);
            servo.setDirection(BioBuzzConfig.FLOWER_ARM_DIRECTION);
        }
        armUp();
    }
    public void armUp() { setPosition(BioBuzzConfig.ARM_UP); }
    public void armDown() { setPosition(BioBuzzConfig.ARM_DOWN); }
    public void setPosition(double position) {
        if (!Double.isFinite(position) || position < 0 || position > 1) {
            throw new IllegalArgumentException("Servo position must be in [0,1]");
        }
        commandedPosition = position;
        if (!dryRun && servo != null) servo.setPosition(position);
    }
    // Commanded position is not a measurement of actual mechanical position.
    public double getCommandedPosition() { return commandedPosition; }
    public Runnable armUpParametric() { armUp(); return null; }
    public Runnable armDownParametric() { armDown(); return null; }
    public Runnable setPositionParametric(double p) { setPosition(p); return null; }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** goBILDA laser sensors in DIGITAL mode. This detects occupancy, not a ball count. */
public final class BioBuzzBallSensors {
    private DigitalChannel first, second;
    private final boolean dryRun = BioBuzzConfig.DRY_RUN;
    private boolean simulatedFirst, simulatedSecond, detectedFirst, detectedSecond;
    private boolean full;
    private double clearSince = Double.NaN;
    public void init(HardwareMap map) {
        if (dryRun) return;
        first = map.get(DigitalChannel.class, BioBuzzConfig.FULL_SENSOR_1);
        second = map.get(DigitalChannel.class, BioBuzzConfig.FULL_SENSOR_2);
        first.setMode(DigitalChannel.Mode.INPUT);
        second.setMode(DigitalChannel.Mode.INPUT);
    }
    public void update(double now) {
        detectedFirst = dryRun ? simulatedFirst : first.getState() == BioBuzzConfig.SENSOR_1_ACTIVE_HIGH;
        detectedSecond = dryRun ? simulatedSecond : second.getState() == BioBuzzConfig.SENSOR_2_ACTIVE_HIGH;
        boolean detected = BioBuzzConfig.FULL_REQUIRES_BOTH
                ? detectedFirst && detectedSecond : detectedFirst || detectedSecond;
        if (detected) { full = true; clearSince = Double.NaN; }
        else {
            if (Double.isNaN(clearSince)) clearSince = now;
            if (now - clearSince >= BioBuzzConfig.FULL_CLEAR_SECONDS) full = false;
        }
    }
    public boolean isFull() { return full; }
    public boolean firstDetected() { return detectedFirst; }
    public boolean secondDetected() { return detectedSecond; }
    public void setSimulatedDetection(boolean first, boolean second) {
        if (!dryRun) throw new IllegalStateException("Simulation is only available in DRY_RUN");
        simulatedFirst = first; simulatedSecond = second;
    }
}
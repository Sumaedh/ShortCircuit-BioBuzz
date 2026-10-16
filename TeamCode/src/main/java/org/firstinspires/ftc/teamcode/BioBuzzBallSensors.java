package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BioBuzzBallSensors {
    DigitalChannel first, second;
    public boolean FIRST_ACTIVE_HIGH = true;
    public boolean SECOND_ACTIVE_HIGH = true;
    public boolean FULL_REQUIRES_BOTH = false;

    public void initSensors(HardwareMap hwMap) {
        first = hwMap.get(DigitalChannel.class, "fullSensor1");
        second = hwMap.get(DigitalChannel.class, "fullSensor2");
        first.setMode(DigitalChannel.Mode.INPUT);
        second.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean firstDetected() { return first.getState() == FIRST_ACTIVE_HIGH; }
    public boolean secondDetected() { return second.getState() == SECOND_ACTIVE_HIGH; }

    public boolean isFull() {
        boolean firstFull = firstDetected();
        boolean secondFull = secondDetected();
        return FULL_REQUIRES_BOTH ? firstFull && secondFull : firstFull || secondFull;
    }
}
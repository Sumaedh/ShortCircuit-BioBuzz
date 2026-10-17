package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors {
    DigitalChannel leftBallSensor, rightBallSensor;
    public boolean FIRST_ACTIVE_HIGH = true;
    public boolean SECOND_ACTIVE_HIGH = true;
    public boolean FULL_REQUIRES_BOTH = false;

    public void initSensors(HardwareMap hwMap) {
        leftBallSensor = hwMap.get(DigitalChannel.class, "leftBallSensor");
        rightBallSensor = hwMap.get(DigitalChannel.class, "rightBallSensor");
        leftBallSensor.setMode(DigitalChannel.Mode.INPUT);
        rightBallSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean firstDetected() { return leftBallSensor.getState() == FIRST_ACTIVE_HIGH; }
    public boolean secondDetected() { return rightBallSensor.getState() == SECOND_ACTIVE_HIGH; }

    public boolean isFull() {
        boolean firstFull = firstDetected();
        boolean secondFull = secondDetected();
        return FULL_REQUIRES_BOTH ? firstFull && secondFull : firstFull || secondFull;
    }

}
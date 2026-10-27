package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {
    public enum StorageMode { DIGITAL, DISTANCE_CM }

    public StorageMode STORAGE_MODE = StorageMode.DISTANCE_CM;
    public boolean FIRST_ACTIVE_HIGH = true;
    public boolean SECOND_ACTIVE_HIGH = true;
    public boolean FULL_REQUIRES_BOTH = false;

    public double FIRST_FULL_CM = 5.0;
    public double SECOND_FULL_CM = 5.0;

    DigitalChannel first, second;
    DistanceSensor firstDistance, secondDistance;

    private StorageMode mappedMode;
    private boolean firstDetected, secondDetected;

    public void initSensors(HardwareMap hwMap) {
        mappedMode = STORAGE_MODE;
        switch (mappedMode) {
            case DIGITAL:
                first = hwMap.get(DigitalChannel.class, "leftBallSensor");
                second = hwMap.get(DigitalChannel.class, "rightBallSensor");
                first.setMode(DigitalChannel.Mode.INPUT);
                second.setMode(DigitalChannel.Mode.INPUT);
                break;
            case DISTANCE_CM:
                firstDistance = hwMap.get(DistanceSensor.class, "leftBallSensor");
                secondDistance = hwMap.get(DistanceSensor.class, "rightBallSensor");
                break;
        }
    }

    public boolean isFull() {
        switch (mappedMode) {
            case DIGITAL:
                firstDetected = first.getState() == FIRST_ACTIVE_HIGH;
                secondDetected = second.getState() == SECOND_ACTIVE_HIGH;
                break;
            case DISTANCE_CM:
                double firstCm = firstDistance.getDistance(DistanceUnit.CM);
                double secondCm = secondDistance.getDistance(DistanceUnit.CM);

                if (Double.isNaN(firstCm) || Double.isInfinite(firstCm) || firstCm < 0 ||
                        Double.isNaN(secondCm) || Double.isInfinite(secondCm) || secondCm < 0) {
                    return false;
                }

                firstDetected = firstCm <= FIRST_FULL_CM;
                secondDetected = secondCm <= SECOND_FULL_CM;
                break;
        }

        return FULL_REQUIRES_BOTH
                ? firstDetected && secondDetected
                : firstDetected || secondDetected;
    }

    public void close() {
    }
}
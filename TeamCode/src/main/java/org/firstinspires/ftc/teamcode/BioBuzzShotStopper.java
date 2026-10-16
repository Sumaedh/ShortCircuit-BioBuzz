package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BioBuzzShotStopper {
    Servo servo;
    public double BLOCKED_POSITION = 0.0;
    public double OPEN_POSITION = 0.0;

    public void initShotStopper(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "shotStopper");
        servo.setDirection(Servo.Direction.FORWARD);
        block();
    }

    public void block() { setPosition(BLOCKED_POSITION); }
    public void open() { setPosition(OPEN_POSITION); }
    public void setPosition(double position) {
        if (servo != null) servo.setPosition(position);
    }

}
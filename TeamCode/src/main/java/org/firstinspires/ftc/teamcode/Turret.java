package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    Servo servo;
    public double LEFT_POSITION = 0.0;
    public double CENTER_POSITION = 0.0;
    public double RIGHT_POSITION = 0.0;

    public void initTurret(HardwareMap hwMap) {
        servo = hwMap.tryGet(Servo.class, "turret");
        if (servo != null) {
            servo.setDirection(Servo.Direction.FORWARD);
            center();
        }
    }
    public boolean available() { return servo != null; }
    public void left() { if (servo != null) servo.setPosition(LEFT_POSITION); }
    public void center() { if (servo != null) servo.setPosition(CENTER_POSITION); }
    public void right() { if (servo != null) servo.setPosition(RIGHT_POSITION); }
}
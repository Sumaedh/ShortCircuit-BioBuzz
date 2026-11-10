package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights {
    Servo light1;
    Servo light2;

    public void initLights(HardwareMap hwMap) {
        light1 = hwMap.get(Servo.class, "light1");
        light2 = hwMap.get(Servo.class, "light2");
    }

    public void notReady() {
        light1.setPosition(0.277);
        light2.setPosition(0.277);
    }

    public void ready() {
        light1.setPosition(0.5);
        light2.setPosition(0.5);
    }
}

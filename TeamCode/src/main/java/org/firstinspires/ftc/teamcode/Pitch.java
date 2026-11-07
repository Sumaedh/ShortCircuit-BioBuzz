package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pitch {

    Servo pitchServo;

    //TODO: FIND THIS
    public double POS_DOWN = 0;

    public void initPitch(HardwareMap hwMap) {
        pitchServo = hwMap.get(Servo.class, "pitchServo");
        pitchDown();
    }

    public void pitchUp() {
        pitchServo.setPosition(0.05);
    }

    public void pitchDown() {
        pitchServo.setPosition(POS_DOWN);
    }

    public void setPitchServo(double pos) {
        pitchServo.setPosition(pos);
    }
}
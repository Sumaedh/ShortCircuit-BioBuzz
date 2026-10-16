package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BioBuzzFlowerArm {
    Servo servo;
    public double UP_POSITION = 0.0;
    public double DOWN_POSITION = 0.0;

    public void initFlowerArm(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "flowerArm");
        servo.setDirection(Servo.Direction.FORWARD);
        armUp();
    }

    public void armUp() { setPosition(UP_POSITION); }
    public void armDown() { setPosition(DOWN_POSITION); }
    public void setPosition(double position) {
        if (servo != null) servo.setPosition(position);
    }

    public Runnable armUpParametric() { return this::armUp; }
    public Runnable armDownParametric() { return this::armDown; }
}
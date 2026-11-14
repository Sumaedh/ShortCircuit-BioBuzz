package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Meet 1 Red 1", group = "Meet1")
public class Meet1Red1 extends Meet1Autobase {

    // TODO: placeholder distances - measure on the field and tune.
    private static final double PARK_FORWARD_IN = 24;  // + forward, - backward
    private static final double PARK_STRAFE_IN  = 12;  // + right,   - left (mirrored from Blue)

    @Override
    protected double parkForwardInches() {
        return PARK_FORWARD_IN;
    }

    @Override
    protected double parkStrafeInches() {
        return PARK_STRAFE_IN;
    }
}
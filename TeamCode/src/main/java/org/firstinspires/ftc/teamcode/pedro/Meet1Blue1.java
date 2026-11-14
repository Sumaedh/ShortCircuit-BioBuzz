package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Meet 1 Blue 1", group = "Meet1")
public class Meet1Blue1 extends Meet1Autobase {

    private static final double PARK_FORWARD_IN = 24;
    private static final double PARK_STRAFE_IN  = -12;
    @Override
    protected double parkForwardInches() {
        return PARK_FORWARD_IN;
    }

    @Override
    protected double parkStrafeInches() {
        return PARK_STRAFE_IN;
    }
}
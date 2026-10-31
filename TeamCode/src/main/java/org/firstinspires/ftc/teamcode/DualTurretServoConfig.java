package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DualTurretServoConfig {
    // Center/neutral position for the mechanism
    public static double ZERO_POSITION = 0.50;

    // Actual servo positions at the physical left and right stops
    // Tune these by moving the turret to each extreme and reading the values
    public static double LEFT_LIMIT_POS = 0.16;
    public static double RIGHT_LIMIT_POS = 0.84;

    // Target turret angle in degrees relative to center
    public static double TARGET_DEGREES = 0.0;

    // This is the total travel range of the turret
    public static double LEFT_LIMIT_DEGREES = -135.0;
    public static double RIGHT_LIMIT_DEGREES = 135.0;

    // Small step used for gamepad tuning
    public static double STEP_DEGREES = 5.0;
}
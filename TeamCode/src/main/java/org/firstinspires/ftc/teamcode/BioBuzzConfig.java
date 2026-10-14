package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/** All robot-specific names and calibration values live here. */
public final class BioBuzzConfig {
    private BioBuzzConfig() { }

    // Preview TeleOp states without mapping or commanding ANY hardware.
    public static boolean DRY_RUN = true;
    // Set true only after verifying wiring, directions, servo travel and speeds.
    public static boolean HARDWARE_SETTINGS_CONFIRMED = false;
    public static boolean AUTO_SETTINGS_CONFIRMED = false;

    public static final String FRONT_LEFT = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT = "backLeft";
    public static final String BACK_RIGHT = "backRight";
    public static final String INTAKE_LEFT = "intakeLeft";
    public static final String INTAKE_RIGHT = "intakeRight";
    public static final String SHOOTER_LEFT = "shooterLeft";
    public static final String SHOOTER_RIGHT = "shooterRight";
    public static final String SHOT_STOPPER = "shotStopper";
    public static final String INTAKE_STOPPER = "intakeStopper";
    public static final String FLOWER_ARM = "flowerArm";
    public static final String FULL_SENSOR_1 = "fullSensor1";
    public static final String FULL_SENSOR_2 = "fullSensor2";

    // Drivetrain directions retain your earlier mecanum setup; verify on this build.
    public static DcMotorSimple.Direction LEFT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    // One transfer/intake motor is reversed, as requested.
    public static DcMotorSimple.Direction INTAKE_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction INTAKE_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    // UNKNOWN until assembly: FORWARD is only a placeholder for both shooters.
    public static DcMotorSimple.Direction SHOOTER_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction SHOOTER_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static Servo.Direction SHOT_STOPPER_DIRECTION = Servo.Direction.FORWARD;
    public static Servo.Direction INTAKE_STOPPER_DIRECTION = Servo.Direction.FORWARD;
    public static Servo.Direction FLOWER_ARM_DIRECTION = Servo.Direction.FORWARD;

    // Zero placeholders are NOT calibrated servo positions. Live validation rejects equal endpoints.
    public static double SHOT_BLOCKED = 0.0;
    public static double SHOT_OPEN = 0.0;
    public static double INTAKE_BLOCKED = 0.0;
    public static double INTAKE_OPEN = 0.0;
    public static double ARM_UP = 0.0;
    public static double ARM_DOWN = 0.0;

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_FEED_POWER = 1.0;
    // Encoder ticks/second, NOT RPM. Measure and tune both motors on the built robot.
    public static double CLOSE_TPS = 0.0;
    public static double FAR_TPS = 0.0;
    public static double SHOOTER_TOLERANCE_TPS = 75.0;
    public static double SHOOTER_READY_SECONDS = 0.20;
    // Keep the Hub's existing velocity PIDF unless measured coefficients are supplied.
    public static boolean USE_CUSTOM_SHOOTER_PIDF = false;
    public static double SHOOTER_P = 0, SHOOTER_I = 0, SHOOTER_D = 0, SHOOTER_F = 0;

    // Assumption: goBILDA 3124-0001-0001, digital mode, HIGH means object detected.
    // Both sensors must observe the fourth-ball/full-storage position, not the intake entrance.
    public static boolean SENSOR_1_ACTIVE_HIGH = true;
    public static boolean SENSOR_2_ACTIVE_HIGH = true;
    // false = either sensor means full; true requires both. Confirm using actual placement.
    public static boolean FULL_REQUIRES_BOTH = false;
    public static double FULL_CLEAR_SECONDS = 0.10;

    // Provisional timing/tolerances; tune before allowing live autonomous.
    public static double ARM_SETTLE_SECONDS = 0.40;
    public static double PICKUP_DWELL_SECONDS = 0.70;
    public static double AUTO_FEED_SECONDS = 1.50;
    public static double AUTO_SHOT_TIMEOUT_SECONDS = 5.0;
    public static double AUTO_PATH_TIMEOUT_SECONDS = 10.0;
    public static double AUTO_POSITION_TOLERANCE_INCHES = 1.0;
    public static double AUTO_HEADING_TOLERANCE_DEGREES = 4.0;
    public static double AUTO_POSE_SETTLE_SECONDS = 0.15;
    // Operational watchdog, not a claim about the game's legal autonomous duration.
    public static double AUTO_WATCHDOG_SECONDS = 30.0;

    public static double now() { return System.nanoTime() * 1e-9; }

    public static void validateLiveHardware() {
        if (DRY_RUN) return;
        if (!HARDWARE_SETTINGS_CONFIRMED) {
            throw new IllegalStateException("Complete BioBuzzConfig calibration before disabling DRY_RUN");
        }
        positions("shot stopper", SHOT_BLOCKED, SHOT_OPEN);
        positions("intake stopper", INTAKE_BLOCKED, INTAKE_OPEN);
        positions("flower arm", ARM_UP, ARM_DOWN);
        positive("CLOSE_TPS", CLOSE_TPS);
        positive("FAR_TPS", FAR_TPS);
        positive("SHOOTER_TOLERANCE_TPS", SHOOTER_TOLERANCE_TPS);
        if (SHOOTER_TOLERANCE_TPS >= Math.min(CLOSE_TPS, FAR_TPS)) {
            throw new IllegalStateException("Shooter tolerance must be smaller than either target speed");
        }
        positive("SHOOTER_READY_SECONDS", SHOOTER_READY_SECONDS);
        positive("FULL_CLEAR_SECONDS", FULL_CLEAR_SECONDS);
        power("INTAKE_POWER", INTAKE_POWER);
        power("TRANSFER_FEED_POWER", TRANSFER_FEED_POWER);
    }

    public static void validateAuto() {
        validateLiveHardware();
        if (DRY_RUN || !AUTO_SETTINGS_CONFIRMED) {
            throw new IllegalStateException("Autonomous needs live configured hardware, Pedro tuning and AUTO_SETTINGS_CONFIRMED");
        }
        positive("ARM_SETTLE_SECONDS", ARM_SETTLE_SECONDS);
        positive("PICKUP_DWELL_SECONDS", PICKUP_DWELL_SECONDS);
        positive("AUTO_FEED_SECONDS", AUTO_FEED_SECONDS);
        positive("AUTO_SHOT_TIMEOUT_SECONDS", AUTO_SHOT_TIMEOUT_SECONDS);
        positive("AUTO_PATH_TIMEOUT_SECONDS", AUTO_PATH_TIMEOUT_SECONDS);
        positive("AUTO_POSITION_TOLERANCE_package org.firstinspires.ftc.teamcode;\n" +
                "\n" +
                "import com.qualcomm.robotcore.hardware.HardwareMap;\n" +
                "import com.qualcomm.robotcore.hardware.Servo;\n" +
                "\n" +
                "/** Positional servo: commands a position, not continuous-rotation power. */\n" +
                "public final class BioBuzzShotStopper {\n" +
                "    private Servo servo;\n" +
                "    private final boolean dryRun = BioBuzzConfig.DRY_RUN;\n" +
                "    private double commandedPosition;\n" +
                "    public void init(HardwareMap map) {\n" +
                "        if (!dryRun) {\n" +
                "            servo = map.get(Servo.class, BioBuzzConfig.SHOT_STOPPER);\n" +
                "            servo.setDirection(BioBuzzConfig.SHOT_STOPPER_DIRECTION);\n" +
                "        }\n" +
                "        block();\n" +
                "    }\n" +
                "    public void block() { setPosition(BioBuzzConfig.SHOT_BLOCKED); }\n" +
                "    public void open() { setPosition(BioBuzzConfig.SHOT_OPEN); }\n" +
                "    public void setPosition(double position) {\n" +
                "        if (!Double.isFinite(position) || position < 0 || position > 1) {\n" +
                "            throw new IllegalArgumentException(\"Servo position must be in [0,1]\");\n" +
                "        }\n" +
                "        commandedPosition = position;\n" +
                "        if (!dryRun && servo != null) servo.setPosition(position);\n" +
                "    }\n" +
                "    // Commanded position is not a measurement of actual mechanical position.\n" +
                "    public double getCommandedPosition() { return commandedPosition; }\n" +
                "    public Runnable blockParametric() { block(); return null; }\n" +
                "    public Runnable openParametric() { open(); return null; }\n" +
                "    public Runnable setPositionParametric(double p) { setPosition(p); return null; }\n" +
                "}INCHES", AUTO_POSITION_TOLERANCE_INCHES);
        positive("AUTO_HEADING_TOLERANCE_DEGREES", AUTO_HEADING_TOLERANCE_DEGREES);
        positive("AUTO_POSE_SETTLE_SECONDS", AUTO_POSE_SETTLE_SECONDS);
        positive("AUTO_WATCHDOG_SECONDS", AUTO_WATCHDOG_SECONDS);
    }

    private static void positions(String name, double a, double b) {
        if (!Double.isFinite(a) || !Double.isFinite(b) || a < 0 || a > 1 || b < 0 || b > 1 || a == b) {
            throw new IllegalStateException("Calibrate distinct positions in [0,1] for " + name);
        }
    }
    private static void positive(String name, double value) {
        if (!Double.isFinite(value) || value <= 0) throw new IllegalStateException("Set a positive " + name);
    }
    private static void power(String name, double value) {
        positive(name, value);
        if (value > 1) throw new IllegalStateException(name + " must be <= 1");
    }
}
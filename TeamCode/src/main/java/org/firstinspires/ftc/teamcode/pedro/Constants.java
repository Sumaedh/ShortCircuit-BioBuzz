package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static final String NOT_READY =
            "AUTO NOT READY: paste tuner output into pedro/Constants.java";

    // Names and directions match Drivetrain.java. Confirm directions with the Mecanum Tuner.
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // TODO: paste the localizer tuner's config above and return it here, for example:
    //   return new PinpointLocalizer(h, localizerConfig);
    private static Localizer createLocalizer(HardwareMap h) {
        return null;
    }

    // TODO: paste the Foresight Tuner's config above and return it here, for example:
    //   return new Foresight(foresightConfig);
    private static Algorithm createAlgorithm() {
        return null;
    }

    /** Returns null until the measured tuner values above are filled in. */
    public static Follower create(HardwareMap h) {
        Localizer localizer = createLocalizer(h);
        Algorithm algorithm = createAlgorithm();
        if (localizer == null || algorithm == null) {
            return null;
        }
        return new Follower(localizer, new Mecanum(h, drivetrainConfig), algorithm);
    }
}

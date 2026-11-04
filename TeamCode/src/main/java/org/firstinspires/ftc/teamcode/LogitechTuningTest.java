package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "BIO-BUZZ C270 Tuning Test", group = "BIO-BUZZ")
public class LogitechTuningTest extends LinearOpMode {
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private static final int START_EXPOSURE_MS = 6;
    private static final int START_GAIN = 250;
    private static final int EXPOSURE_STEP_MS = 1;
    private static final int GAIN_STEP = 10;
    private static final int WHITE_BALANCE_STEP_K = 250;
    private static final double DISTANCE_STEP_IN = 0.5;
    private static final float[] DECIMATION = {1f, 2f, 3f};

    private static final int SAMPLE_COUNT = 60;
    private static final int MAX_FRAMES_PER_RUN = SAMPLE_COUNT * 5;

    private enum TestState { LIVE, SAMPLING, RESULTS }

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WhiteBalanceControl whiteBalanceControl;

    private TestState state = TestState.LIVE;
    private String message = "";

    private boolean manualExposure = true;
    private int exposureMs = START_EXPOSURE_MS;
    private int minExposureMs = 1;
    private int maxExposureMs = 100;
    private int gain = START_GAIN;
    private int minGain = 0;
    private int maxGain = 255;

    private boolean whiteBalanceSupported = false;
    private boolean manualWhiteBalance = false;
    private int whiteBalanceK = 4000;
    private int minWhiteBalanceK = 2000;
    private int maxWhiteBalanceK = 6500;

    private int decimationIndex = 1;
    private double trueDistanceIn = 24.0;

    private String targetKey = null;
    private int framesChecked = 0;
    private final List<Double> rangeSamples = new ArrayList<>();
    private final List<Double> bearingSamples = new ArrayList<>();
    private final List<Double> yawSamples = new ArrayList<>();

    private Stats rangeStats;
    private Stats bearingStats;
    private Stats yawStats;
    private double detectionRate;

    private final Button dpadUp = new Button();
    private final Button dpadDown = new Button();
    private final Button dpadLeft = new Button();
    private final Button dpadRight = new Button();
    private final Button leftBumper = new Button();
    private final Button rightBumper = new Button();
    private final Button buttonX = new Button();
    private final Button buttonY = new Button();
    private final Button buttonA = new Button();
    private final Button buttonB = new Button();
    private final Button leftStick = new Button();

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(DECIMATION[decimationIndex]);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        waitForStreaming();
        if (isStopRequested()) {
            portal.close();
            return;
        }
        setupCameraControls();

        while (opModeInInit()) {
            telemetry.addLine("C270 ready. Press START.");
            telemetry.addLine("Point the camera at an AprilTag.");
            addCameraTelemetry();
            telemetry.update();
            sleep(50);
        }

        while (opModeIsActive()) {
            updateButtons();
            handleInput();

            if (state == TestState.SAMPLING) {
                collectSample();
            }

            switch (state) {
                case LIVE:     showLive();     break;
                case SAMPLING: showSampling(); break;
                case RESULTS:  showResults();  break;
            }
            telemetry.update();
            sleep(10);
        }

        portal.close();
    }

    private void waitForStreaming() {
        while (!isStopRequested() && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting... (%s)", portal.getCameraState());
            telemetry.update();
            sleep(20);
        }
    }

    private void setupCameraControls() {
        exposureControl = portal.getCameraControl(ExposureControl.class);
        gainControl = portal.getCameraControl(GainControl.class);
        whiteBalanceControl = portal.getCameraControl(WhiteBalanceControl.class);

        if (exposureControl != null) {
            minExposureMs = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposureMs = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        }
        if (gainControl != null) {
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
        if (whiteBalanceControl != null) {
            minWhiteBalanceK = whiteBalanceControl.getMinWhiteBalanceTemperature();
            maxWhiteBalanceK = whiteBalanceControl.getMaxWhiteBalanceTemperature();
            whiteBalanceSupported = maxWhiteBalanceK > minWhiteBalanceK;
            if (whiteBalanceSupported) {
                whiteBalanceK = clamp(whiteBalanceControl.getWhiteBalanceTemperature(),
                        minWhiteBalanceK, maxWhiteBalanceK);
            }
        }

        exposureMs = clamp(exposureMs, minExposureMs, maxExposureMs);
        gain = clamp(gain, minGain, maxGain);
        applyExposure();
    }

    private void applyExposure() {
        if (exposureControl == null) {
            return;
        }
        if (manualExposure) {
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            sleep(20);
            if (gainControl != null) {
                gainControl.setGain(gain);
                sleep(20);
            }
        } else {
            exposureControl.setMode(ExposureControl.Mode.Auto);
            sleep(50);
        }
    }

    private void applyWhiteBalance() {
        if (!whiteBalanceSupported) {
            return;
        }
        if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
            sleep(50);
        }
        whiteBalanceControl.setWhiteBalanceTemperature(whiteBalanceK);
        sleep(20);
    }

    private void updateButtons() {
        dpadUp.update(gamepad1.dpad_up);
        dpadDown.update(gamepad1.dpad_down);
        dpadLeft.update(gamepad1.dpad_left);
        dpadRight.update(gamepad1.dpad_right);
        leftBumper.update(gamepad1.left_bumper);
        rightBumper.update(gamepad1.right_bumper);
        buttonX.update(gamepad1.x);
        buttonY.update(gamepad1.y);
        buttonA.update(gamepad1.a);
        buttonB.update(gamepad1.b);
        leftStick.update(gamepad1.left_stick_button);
    }

    private void handleInput() {
        if (buttonX.pressed()) {
            switch (state) {
                case LIVE:     startSampling(); break;
                case SAMPLING: state = TestState.LIVE; message = "Run cancelled."; break;
                case RESULTS:  state = TestState.LIVE; message = ""; break;
            }
            return;
        }
        if (state == TestState.SAMPLING) {
            return;
        }

        if (buttonY.pressed()) {
            trueDistanceIn += DISTANCE_STEP_IN;
        }
        if (buttonA.pressed()) {
            trueDistanceIn = Math.max(DISTANCE_STEP_IN, trueDistanceIn - DISTANCE_STEP_IN);
        }
        if (buttonB.pressed()) {
            decimationIndex = (decimationIndex + 1) % DECIMATION.length;
            aprilTag.setDecimation(DECIMATION[decimationIndex]);
        }

        boolean exposureChanged = false;
        if (leftStick.pressed()) {
            manualExposure = !manualExposure;
            exposureChanged = true;
        }
        if (manualExposure) {
            if (dpadUp.pressed()) {
                exposureMs = clamp(exposureMs + EXPOSURE_STEP_MS, minExposureMs, maxExposureMs);
                exposureChanged = true;
            }
            if (dpadDown.pressed()) {
                exposureMs = clamp(exposureMs - EXPOSURE_STEP_MS, minExposureMs, maxExposureMs);
                exposureChanged = true;
            }
            if (dpadRight.pressed()) {
                gain = clamp(gain + GAIN_STEP, minGain, maxGain);
                exposureChanged = true;
            }
            if (dpadLeft.pressed()) {
                gain = clamp(gain - GAIN_STEP, minGain, maxGain);
                exposureChanged = true;
            }
        }
        if (exposureChanged) {
            applyExposure();
        }

        if (whiteBalanceSupported && (rightBumper.pressed() || leftBumper.pressed())) {
            int step = rightBumper.pressed() ? WHITE_BALANCE_STEP_K : -WHITE_BALANCE_STEP_K;
            whiteBalanceK = clamp(whiteBalanceK + step, minWhiteBalanceK, maxWhiteBalanceK);
            manualWhiteBalance = true;
            applyWhiteBalance();
        }
    }

    private void startSampling() {
        targetKey = null;
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            String key = keyFor(detection);
            if (key != null) {
                targetKey = key;
                break;
            }
        }
        if (targetKey == null) {
            message = "No known AprilTag in view. Aim at a tag first.";
            return;
        }

        rangeSamples.clear();
        bearingSamples.clear();
        yawSamples.clear();
        framesChecked = 0;
        message = "";
        aprilTag.getFreshDetections();
        state = TestState.SAMPLING;
    }

    private void collectSample() {
        List<AprilTagDetection> fresh = aprilTag.getFreshDetections();
        if (fresh == null) {
            return;
        }
        framesChecked++;

        for (AprilTagDetection detection : fresh) {
            if (targetKey.equals(keyFor(detection)) && detection.ftcPose != null) {
                rangeSamples.add(detection.ftcPose.range);
                bearingSamples.add(detection.ftcPose.bearing);
                yawSamples.add(detection.ftcPose.yaw);
                break;
            }
        }

        if (rangeSamples.size() >= SAMPLE_COUNT) {
            finishSampling();
        } else if (framesChecked >= MAX_FRAMES_PER_RUN) {
            if (rangeSamples.size() >= 2) {
                message = "Tag was lost often. Results use fewer samples.";
                finishSampling();
            } else {
                message = "Tag was not seen enough to measure. Check lighting and aim.";
                state = TestState.LIVE;
            }
        }
    }

    private void finishSampling() {
        rangeStats = new Stats(rangeSamples);
        bearingStats = new Stats(bearingSamples);
        yawStats = new Stats(yawSamples);
        detectionRate = 100.0 * rangeSamples.size() / framesChecked;
        state = TestState.RESULTS;
    }

    private void addCameraTelemetry() {
        telemetry.addData("Camera", "%s  %.1f FPS  %dx%d",
                portal.getCameraState(), portal.getFps(), CAMERA_WIDTH, CAMERA_HEIGHT);
        if (manualExposure) {
            telemetry.addData("Exposure", "MANUAL %d ms  (%d-%d)", exposureMs, minExposureMs, maxExposureMs);
            telemetry.addData("Gain", "%d  (%d-%d)", gain, minGain, maxGain);
        } else {
            telemetry.addData("Exposure", "AUTO");
        }
        if (!whiteBalanceSupported) {
            telemetry.addData("White balance", "Not supported");
        } else if (manualWhiteBalance) {
            telemetry.addData("White balance", "MANUAL %d K  (%d-%d)", whiteBalanceK, minWhiteBalanceK, maxWhiteBalanceK);
        } else {
            telemetry.addData("White balance", "AUTO");
        }
        telemetry.addData("Decimation", "%.0f", DECIMATION[decimationIndex]);
    }

    @SuppressLint("DefaultLocale")
    private void showLive() {
        addCameraTelemetry();
        telemetry.addData("True distance", "%.1f in  (lens to tag center)", trueDistanceIn);
        if (!message.isEmpty()) {
            telemetry.addLine(message);
        }
        telemetry.addLine();

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("Tags seen", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("%s  (%d%% of cluster found)",
                        keyFor(detection), cluster.percentClusterFound));
            } else {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.metadata == null) {
                    telemetry.addLine(String.format("Tag %d unknown (no pose)", single.id));
                    continue;
                }
                telemetry.addLine(keyFor(detection));
            }
            if (detection.ftcPose != null) {
                telemetry.addLine(String.format("  Range %.2f in  Bearing %.1f deg  Yaw %.1f deg",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw));
                telemetry.addLine(String.format("  X %.2f  Y %.2f  Z %.2f in",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            }
        }

        telemetry.addLine();
        telemetry.addLine("Dpad Up/Down: exposure   Dpad L/R: gain");
        telemetry.addLine("LB/RB: white balance   L-stick click: auto/manual exposure");
        telemetry.addLine("Y/A: true distance +/-   B: decimation");
        telemetry.addLine("X: run precision test (" + SAMPLE_COUNT + " frames)");
    }

    private void showSampling() {
        telemetry.addLine("SAMPLING - hold the robot still");
        telemetry.addData("Target", targetKey);
        telemetry.addData("Samples", "%d / %d", rangeSamples.size(), SAMPLE_COUNT);
        telemetry.addData("Frames checked", framesChecked);
        telemetry.addLine("X: cancel");
    }

    private void showResults() {
        double error = rangeStats.mean - trueDistanceIn;
        double errorPercent = 100.0 * error / trueDistanceIn;

        telemetry.addLine("PRECISION TEST RESULTS");
        if (!message.isEmpty()) {
            telemetry.addLine(message);
        }
        telemetry.addData("Target", targetKey);
        telemetry.addData("Samples", "%d  (detected in %.0f%% of frames)", rangeStats.count, detectionRate);
        telemetry.addLine();

        telemetry.addLine("ACCURACY (vs true distance)");
        telemetry.addData("True distance", "%.2f in", trueDistanceIn);
        telemetry.addData("Measured mean", "%.2f in", rangeStats.mean);
        telemetry.addData("Error", "%+.2f in  (%+.1f%%)", error, errorPercent);
        telemetry.addData("Rating", accuracyRating(Math.abs(errorPercent)));
        telemetry.addLine();

        telemetry.addLine("PRECISION (frame-to-frame jitter)");
        telemetry.addData("Range", "SD %.3f in  spread %.3f in", rangeStats.stdDev, rangeStats.max - rangeStats.min);
        telemetry.addData("Bearing", "mean %.2f  SD %.3f deg", bearingStats.mean, bearingStats.stdDev);
        telemetry.addData("Yaw", "mean %.2f  SD %.3f deg", yawStats.mean, yawStats.stdDev);
        telemetry.addData("Rating", precisionRating(rangeStats.stdDev));
        telemetry.addLine();

        addCameraTelemetry();
        telemetry.addLine();
        telemetry.addLine("X: back to live view");
    }

    private String accuracyRating(double absErrorPercent) {
        if (absErrorPercent <= 2.0) return "Excellent (within 2%)";
        if (absErrorPercent <= 5.0) return "Good (within 5%)";
        return "Check: re-measure true distance or calibration";
    }

    private String precisionRating(double rangeStdDevIn) {
        if (rangeStdDevIn <= 0.10) return "Excellent (SD <= 0.10 in)";
        if (rangeStdDevIn <= 0.25) return "Good (SD <= 0.25 in)";
        return "Noisy: lower exposure, raise gain, or decimation 1";
    }

    private static String keyFor(AprilTagDetection detection) {
        if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            return single.metadata != null ? "Tag " + single.id + " " + single.metadata.name : null;
        }
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return cluster.metadata != null ? "Cluster " + cluster.metadata.name : null;
        }
        return null;
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    private static class Stats {
        final int count;
        final double mean;
        final double stdDev;
        final double min;
        final double max;

        Stats(List<Double> values) {
            count = values.size();
            double sum = 0;
            double lo = Double.MAX_VALUE;
            double hi = -Double.MAX_VALUE;
            for (double v : values) {
                sum += v;
                lo = Math.min(lo, v);
                hi = Math.max(hi, v);
            }
            mean = count > 0 ? sum / count : 0;

            double squares = 0;
            for (double v : values) {
                squares += (v - mean) * (v - mean);
            }
            stdDev = count > 1 ? Math.sqrt(squares / (count - 1)) : 0;
            min = count > 0 ? lo : 0;
            max = count > 0 ? hi : 0;
        }
    }

    private static class Button {
        private boolean last = false;
        private boolean pressed = false;

        void update(boolean current) {
            pressed = current && !last;
            last = current;
        }

        boolean pressed() {
            return pressed;
        }
    }
}
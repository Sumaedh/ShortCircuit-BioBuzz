package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Sensors {
    public enum StorageMode { DIGITAL, DISTANCE_CM }
    public enum StorageState { NOT_STARTED, CLEAR, FULL, FAULT }
    public enum CameraState { NOT_STARTED, MISSING, WAITING, NO_TARGET, TARGET, STALE, PAUSED }

    public StorageMode STORAGE_MODE = StorageMode.DISTANCE_CM;
    public boolean FIRST_ACTIVE_HIGH = true;
    public boolean SECOND_ACTIVE_HIGH = true;
    public boolean FULL_REQUIRES_BOTH = false;

    public double FIRST_FULL_CM = 5.0;
    public double SECOND_FULL_CM = 5.0;

    public int LIMELIGHT_PIPELINE = 0;
    public int LIMELIGHT_POLL_HZ = 100;
    public double MAX_CAMERA_AGE_MS = 250;

    DigitalChannel first, second;
    DistanceSensor firstDistance, secondDistance;

    private StorageMode mappedMode;
    private Limelight3A limelight;
    private AprilTagProcessor aprilTags;
    private VisionPortal portal;
    private LLResult latestResult;
    private List<AprilTagDetection> detections = Collections.emptyList();
    private long webcamFrameNanos;
    private boolean firstDetected, secondDetected;
    private double firstCm = Double.NaN, secondCm = Double.NaN;
    private StorageState storageState = StorageState.NOT_STARTED;
    private CameraState limelightState = CameraState.NOT_STARTED;
    private CameraState webcamState = CameraState.NOT_STARTED;
    private boolean pipelineAccepted;

    public void initSensors(HardwareMap hwMap) {
        initStorage(hwMap);
        initLimelight(hwMap);
        initLogitech(hwMap);
    }

    public void initStorage(HardwareMap hwMap) {
        mappedMode = STORAGE_MODE;
        switch (mappedMode) {
            case DIGITAL:
                first = hwMap.get(DigitalChannel.class, "leftBallSensor");
                second = hwMap.get(DigitalChannel.class, "rightBallSensor");
                first.setMode(DigitalChannel.Mode.INPUT);
                second.setMode(DigitalChannel.Mode.INPUT);
                break;
            case DISTANCE_CM:
                firstDistance = hwMap.get(DistanceSensor.class, "leftBallSensor");
                secondDistance = hwMap.get(DistanceSensor.class, "rightBallSensor");
                break;
        }
        updateStorage();
    }

    public void initLimelight(HardwareMap hwMap) {
        limelight = hwMap.tryGet(Limelight3A.class, "limelight");
        if (limelight == null) {
            limelightState = CameraState.MISSING;
            return;
        }
        limelight.setPollRateHz(Math.max(1, Math.min(250, LIMELIGHT_POLL_HZ)));
        selectPipeline(LIMELIGHT_PIPELINE);
        limelight.start();
        limelightState = CameraState.WAITING;
    }

    public void initLogitech(HardwareMap hwMap) {
        WebcamName webcam = hwMap.tryGet(WebcamName.class, "Webcam 1");
        if (webcam == null) {
            webcamState = CameraState.MISSING;
            return;
        }

        aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(new AprilTagLibrary.Builder().build())
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTags)
                .enableLiveView(true)
                .build();

        webcamState = CameraState.WAITING;
    }

    public boolean selectPipeline(int pipeline) {
        LIMELIGHT_PIPELINE = Math.max(0, Math.min(9, pipeline));
        latestResult = null;
        pipelineAccepted = limelight != null && limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        return pipelineAccepted;
    }

    public void setLimelightRunning(boolean running) {
        if (limelight == null) return;
        if (running) limelight.start();
        else limelight.pause();
    }

    public void update() {
        if (mappedMode != null) updateStorage();
        if (limelight != null) updateLimelight();
        if (portal != null) updateLogitech();
    }

    private void updateStorage() {
        switch (mappedMode) {
            case DIGITAL:
                firstDetected = first.getState() == FIRST_ACTIVE_HIGH;
                secondDetected = second.getState() == SECOND_ACTIVE_HIGH;
                break;
            case DISTANCE_CM:
                firstCm = firstDistance.getDistance(DistanceUnit.CM);
                secondCm = secondDistance.getDistance(DistanceUnit.CM);

                if (!validDistance(firstCm) || !validDistance(secondCm)) {
                    storageState = StorageState.FAULT;
                    firstDetected = false;
                    secondDetected = false;
                    return;
                }

                firstDetected = firstCm <= FIRST_FULL_CM;
                secondDetected = secondCm <= SECOND_FULL_CM;
                break;
        }

        boolean full = FULL_REQUIRES_BOTH
                ? firstDetected && secondDetected
                : firstDetected || secondDetected;

        storageState = full ? StorageState.FULL : StorageState.CLEAR;
    }

    private boolean validDistance(double cm) {
        return !Double.isNaN(cm) && !Double.isInfinite(cm) && cm >= 0;
    }

    private void updateLimelight() {
        latestResult = limelight.getLatestResult();

        if (!limelight.isRunning()) {
            limelightState = CameraState.PAUSED;
        } else if (!limelight.isConnected() || latestResult == null) {
            limelightState = CameraState.WAITING;
        } else if (latestResult.getStaleness() > MAX_CAMERA_AGE_MS) {
            limelightState = CameraState.STALE;
        } else if (latestResult.getPipelineIndex() != LIMELIGHT_PIPELINE) {
            limelightState = CameraState.WAITING;
        } else {
            limelightState = latestResult.isValid() ? CameraState.TARGET : CameraState.NO_TARGET;
        }
    }

    private void updateLogitech() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            webcamState = CameraState.WAITING;
            detections = Collections.emptyList();
            return;
        }

        ArrayList<AprilTagDetection> fresh = aprilTags.getFreshDetections();
        if (fresh != null) {
            detections = new ArrayList<>(fresh);
            webcamFrameNanos = System.nanoTime();
        }

        if (webcamFrameNanos == 0) {
            webcamState = CameraState.WAITING;
        } else if (((System.nanoTime() - webcamFrameNanos) * 1e-6) > MAX_CAMERA_AGE_MS) {
            webcamState = CameraState.STALE;
            detections = Collections.emptyList();
        } else {
            webcamState = detections.isEmpty() ? CameraState.NO_TARGET : CameraState.TARGET;
        }
    }

    public boolean isFull() {
        return storageState == StorageState.FULL;
    }

    public boolean storageReady() {
        return storageState == StorageState.CLEAR || storageState == StorageState.FULL;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Storage", storageState);
        telemetry.addData("Storage input mode", mappedMode);
        telemetry.addData("Full sensor 1 / 2", firstDetected + " / " + secondDetected);

        if (mappedMode == StorageMode.DISTANCE_CM) {
            telemetry.addData("Sensor 1 / 2 cm", firstCm + " / " + secondCm);
        }

        telemetry.addData("Limelight", limelightState);
        telemetry.addData("Requested pipeline / accepted", LIMELIGHT_PIPELINE + " / " + pipelineAccepted);

        if (latestResult != null) {
            telemetry.addData("Limelight age ms", latestResult.getStaleness());
            telemetry.addData("Actual pipeline", latestResult.getPipelineIndex());
        }

        if (limelightState == CameraState.TARGET) {
            telemetry.addData("Limelight tx / ty degrees", latestResult.getTx() + " / " + latestResult.getTy());
            telemetry.addData("Limelight area percent", latestResult.getTa());
        }

        telemetry.addData("Logitech", webcamState);

        if (portal != null) telemetry.addData("Webcam stream", portal.getCameraState());

        for (AprilTagDetection tag : detections) {
            telemetry.addData("Webcam tag ID", tag);
        }
    }

    public void close() {
        if (limelight != null) limelight.stop();
        if (portal != null) portal.close();
    }
}
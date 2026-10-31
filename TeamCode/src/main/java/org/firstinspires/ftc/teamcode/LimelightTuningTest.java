package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Tuning Test", group = "BIOBUZZ")
public class LimelightTuningTest extends OpMode {
    private Limelight3A limelight;
    private int currentPipeline = 0;
    private boolean previousX = false;
    private boolean previousB = false;

    @Override
    public void init() {
        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(currentPipeline);
            limelight.start();
        }
    }

    @Override
    public void loop() {
        if (limelight == null) {
            telemetry.addData("Status", "Limelight not found!");
            telemetry.update();
            return;
        }

        if (gamepad1.x && !previousX) {
            currentPipeline = (currentPipeline + 1) % 9;
            limelight.pipelineSwitch(currentPipeline);
        }
        if (gamepad1.b && !previousB) {
            currentPipeline = (currentPipeline - 1 + 9) % 9;
            limelight.pipelineSwitch(currentPipeline);
        }

        previousX = gamepad1.x;
        previousB = gamepad1.b;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("Pipeline", currentPipeline);
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("Target Valid", true);
        } else {
            telemetry.addData("Pipeline", currentPipeline);
            telemetry.addData("Target Valid", false);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
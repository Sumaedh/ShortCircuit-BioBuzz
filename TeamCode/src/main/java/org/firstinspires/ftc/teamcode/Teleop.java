package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Teleop extends OpMode {

    Drivetrain dt = new Drivetrain();
    IntakeTransferArm ita = new IntakeTransferArm();
    Shooter sh = new Shooter();
    Sensors sensors = new Sensors();

    enum DriveState { NORMAL, SLOW }
    enum IntakeState { OFF, ON, REVERSE, FULL, FEED }
    enum ShooterState { OFF, ON }
    enum ArmState { UP, INTAKEFLOWER, BLOCK }

    //TODO: INVERSE KINEMATICS AND TURRET
    //TODO: LIMELIGHT

    DriveState driveState = DriveState.NORMAL;
    IntakeState intakeState = IntakeState.OFF;
    ShooterState shooterState = ShooterState.OFF;
    ArmState armState = ArmState.UP;

    @Override
    public void init() {
        dt.initDrivetrain(hardwareMap);
        ita.initIntakeTransfer(hardwareMap);
        sh.initShooter(hardwareMap);
        sensors.initSensors(hardwareMap);
    }

    @Override
    public void loop() {
        switch (driveState) {
            case NORMAL:
                dt.driveMecanum(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                );
                if (gamepad1.left_trigger > 0.75) {
                    driveState = DriveState.SLOW;
                }
                break;
            case SLOW:
                dt.driveMecanumSlower(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                );
                if (gamepad1.left_trigger < 0.75) {
                    driveState = DriveState.NORMAL;
                }
                break;
            default:
                driveState = DriveState.NORMAL;
        }

        // TODO: INVERSE KINEMATICS OF ARM

        // TODO: WRITE SHOOTER STATES

        // TODO TURRET STATES
    }
}

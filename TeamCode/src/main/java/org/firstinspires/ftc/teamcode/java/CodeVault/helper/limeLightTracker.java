package org.firstinspires.ftc.teamcode.java.CodeVault.helper;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class limeLightTracker extends LinearOpMode {

    private Limelight3A limelight;


    @Override
    public void runOpMode() {
        // --- Initialize motors ---
        fieldCentric drive = new fieldCentric(hardwareMap,"fl","fr","bl","br");
        IMU imu = imuCall.initIMU(hardwareMap, "imu");


        // --- Initialize Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        double threshold = 0.2;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // --- Main loop ---
        while (opModeIsActive()) {
            // --- Read Gamepad Inputs ---
            double y  = -gamepad1.left_stick_y;   // forward/back
            double x  =  gamepad1.left_stick_x;   // strafe
            double rx =  gamepad1.right_stick_x;  // rotation
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

// --- Deadzone Filtering ---
            if (Math.abs(y) < 0.2) y = 0;
            if (Math.abs(x) < 0.2) x = 0;
            if (Math.abs(rx) < 0.2) rx = 0;

// --- AprilTag Tracking ---
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // horizontal offset
                telemetry.addData("Target X", tx);

                // Proportional control for turning toward tag
                double kP = 0.015;
                double turnCorrection = tx * kP;

                // Optional: Clamp turn to prevent over-rotation
                turnCorrection = Math.max(-0.3, Math.min(0.3, turnCorrection));

                // Only apply correction if driver isn't manually turning
                if (Math.abs(rx) < 0.05) rx = turnCorrection;

                telemetry.addData("Turn Correction", turnCorrection);
            } else {
                telemetry.addLine("No Target Found");
            }

// --- Drive ---
            drive.drive(x, y, rx, botHeading);
            telemetry.update();


        }
    }
}

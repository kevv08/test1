package org.firstinspires.ftc.teamcode.java.CodeVault.auto.finalAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Alliance Auto", group = "Examples")
class RedAllianceAuto extends BlueAllianceAuto {

    @Override
    public void init() {
        // Initialize everything first
        super.init();

        // NOW MIRROR ALL POSES FOR RED ALLIANCE
        // Pedro's mirror() method flips Y coordinate and heading
        startPose = startPose.mirror();
        scorePose = scorePose.mirror();
        prePickUpPose1 = prePickUpPose1.mirror();
        pickUpPose1 = pickUpPose1.mirror();
        prePickUpPose2 = prePickUpPose2.mirror();
        pickUpPose2 = pickUpPose2.mirror();

        // Update follower with mirrored starting pose
        follower.setStartingPose(startPose);

        // Rebuild paths with mirrored poses
        buildPaths();

        telemetry.addLine("Red Alliance Auto Initialized (MIRRORED)");
        telemetry.update();
    }
}
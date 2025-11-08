package org.firstinspires.ftc.teamcode.java.CodeVault.auto.randomAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.java.CodeVault.auto.Constants;

/**
 * Example Auto for Pedro Pathing that actually drives forward and follows multiple paths.
 * Tested with standard mecanum setup.
 */
@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState = 0;

    // --- Define start and goal poses ---
    private final Pose startPose   = new Pose(0, 0, Math.toRadians(0));   // starting position
    private final Pose scorePose   = new Pose(24, 0, Math.toRadians(0));  // drive forward 24 inches
    private final Pose pickup1Pose = new Pose(24, 24, Math.toRadians(90)); // turn + strafe
    private final Pose returnPose  = new Pose(0, 0, Math.toRadians(180)); // back to start

    // Paths
    private Path moveForward;
    private PathChain goToPickup, returnToStart;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // --- Initialize timers ---
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // --- Create follower and set starting pose ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // --- Build paths ---
        buildPaths();

        telemetry.addLine("Initialization Complete ✅");
        telemetry.update();
    }

    public void buildPaths() {
        // Simple forward movement path
        moveForward = new Path(new BezierLine(startPose, scorePose));
        moveForward.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // PathChain to pickup
        goToPickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Return back to start
        returnToStart = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, returnPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), returnPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        telemetry.addLine("Autonomous Started!");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();         // must run every cycle
        autonomousPathUpdate();    // update logic

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Autonomous Complete");
        telemetry.update();
    }

    // ---------------- PATH STATE MACHINE ----------------
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(moveForward);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(goToPickup, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(returnToStart, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    telemetry.addLine("All paths complete ✅");
                    telemetry.update();
                    setPathState(-1); // stop state
                }
                break;
        }
    }

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}

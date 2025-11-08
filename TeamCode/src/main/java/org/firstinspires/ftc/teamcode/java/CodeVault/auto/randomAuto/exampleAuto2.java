package org.firstinspires.ftc.teamcode.java.CodeVault.auto.randomAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.CodeVault.auto.Constants;

@Autonomous
public class exampleAuto2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;

    // Hardware references
    private DcMotor intakeLeft, intakeRight;
    private DcMotor shooterLeft, shooterRight;
    private Servo upchargeLeft, upchargeRight;

    // --- Define start and goal poses ---
    private final Pose startPose      = new Pose(22.123, 124, Math.toRadians(144));
    private final Pose scorePose      = new Pose(51.8, 94.7, Math.toRadians(135));
    private final Pose prePickUpPose1 = new Pose(52.3, 33, Math.toRadians(180));
    private final Pose pickUpPose1    = new Pose(14.8, 33, Math.toRadians(180));
    private final Pose prePickUpPose2 = new Pose(51.8, 57.9, Math.toRadians(180));
    private final Pose pickUpPose2    = new Pose(14.7, 57.9, Math.toRadians(180));

    // Paths

    private PathChain scorePreLoad,goToGrabPickup1,grabPickup1, scorePickup1,goToGrabPickup2, grabPickup2, scorePickup2;

    @Override
    public void init() {
        // Initialize timers and follower
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize hardware
        intakeLeft  = hardwareMap.get(DcMotor.class, "il");
        intakeRight = hardwareMap.get(DcMotor.class, "ir");
        shooterRight = hardwareMap.get(DcMotor.class, "sr");
        shooterLeft  = hardwareMap.get(DcMotor.class, "sl");
        upchargeLeft  = hardwareMap.get(Servo.class, "upLeft");
        upchargeRight = hardwareMap.get(Servo.class, "upRight");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        buildPaths();

        telemetry.addLine("Auto Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        telemetry.addLine("Auto Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        telemetry.addLine("Auto Finished");
        telemetry.update();
    }

    // ------------------- PATH BUILDING -------------------
    private void buildPaths() {
        PathChain scorePreload = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                .build(); // ✅ returns a Path


        goToGrabPickup1 = follower
                .pathBuilder()
                .addPath(new BezierLine(scorePose, prePickUpPose1))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

       grabPickup1 = follower
                .pathBuilder()
                .addPath(new BezierLine(prePickUpPose1, pickUpPose1))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        scorePickup1 = follower
                .pathBuilder()
                .addPath(new BezierLine(pickUpPose1, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        goToGrabPickup2 = follower
                .pathBuilder()
                .addPath(new BezierLine(scorePose, prePickUpPose2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
        grabPickup2 = follower
                .pathBuilder()
                .addPath(new BezierLine(prePickUpPose2, pickUpPose2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
        scorePickup2 = follower
                .pathBuilder()
                .addPath(new BezierLine(pickUpPose2, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    // ------------------- PATH EXECUTION -------------------
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreLoad);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    performScoreAction();
                    follower.followPath(goToGrabPickup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (follower.getFollowingPathChain()) {
                    runIntakeDuringPickup();
                } else if (!follower.isBusy()) {
                    stopIntake();
                    performPickupAction();
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    performScoreAction();
                    follower.followPath(scorePickup1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (follower.getFollowingPathChain()) {
                    runIntakeDuringPickup();
                } else if (!follower.isBusy()) {
                    stopIntake();
                    performPickupAction();
                    follower.followPath(goToGrabPickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    performScoreAction();
                    setPathState(-1);
                }
                break;
        }
    }

    // ------------------- ACTION METHODS -------------------
    private void performScoreAction() {
        // Wind-up shooter motors before firing
        shooterLeft.setPower(1);
        shooterRight.setPower(1);

        actionTimer.resetTimer();
        while (actionTimer.getElapsedTimeSeconds() < 1.5) {
            // Wait for motors to spin up
        }

        // Activate servos to shoot
        upchargeLeft.setPosition(0.25);
        upchargeRight.setPosition(0.0);

        actionTimer.resetTimer();
        while (actionTimer.getElapsedTimeSeconds() < 1.5) {
            // Shooting period
        }

        shooterLeft.setPower(0);
        shooterRight.setPower(0);

        // Reset servos
        upchargeLeft.setPosition(0.0);
        upchargeRight.setPosition(0.25);
    }

    private void performPickupAction() {
        // Intake briefly to ensure capture
        intakeLeft.setPower(1);
        intakeRight.setPower(1);

        actionTimer.resetTimer();
        while (actionTimer.getElapsedTimeSeconds() < 3.0) {
            // Intake running
        }

        stopIntake();
    }

    private void runIntakeDuringPickup() {
        intakeLeft.setPower(1);
        intakeRight.setPower(1);
    }

    private void stopIntake() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    // ------------------- STATE MANAGEMENT -------------------
    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

package org.firstinspires.ftc.teamcode.java.CodeVault.auto.randomAuto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.java.CodeVault.auto.Constants;

@Autonomous(name = "testAuto1", group = "Examples")
public class tesAuto1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private DcMotor intakeLeft, intakeRight;
    private DcMotor shooterLeft, shooterRight;



    private Path scorePreload;
    private final Pose startPose = new Pose(22.123, 124, Math.toRadians(144));
    private final Pose scorePose = new Pose(51.8, 94.7, Math.toRadians(135));
    private final Pose prePickUpPose1 = new Pose(52.3, 33, Math.toRadians(180));
    private final Pose pickUpPose1 = new Pose(14.8, 33, Math.toRadians(180));
    private final Pose prePickUpPose2 = new Pose(51.8, 57.9, Math.toRadians(180));
    private final Pose pickUpPose2 = new Pose(14.7, 57.9, Math.toRadians(180));

    private PathChain goToPickup1, grabPickup1, scorePickup1, goToPickup2, grabPickup2, scorePickup2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickUpPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickUpPose1.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prePickUpPose1, pickUpPose1))
                .setLinearHeadingInterpolation(prePickUpPose1.getHeading(), pickUpPose1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose1, scorePose))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickUpPose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickUpPose2.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(prePickUpPose2, pickUpPose2))
                .setLinearHeadingInterpolation(prePickUpPose2.getHeading(), pickUpPose2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                performScoreAction();
                setPathState(1);
                break;
            case 1:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToPickup1, true);
                    performPickupActionDuringPath();
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup1, true);
                    performScoreAction();
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup1, true);
                    performPickupActionDuringPath();
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToPickup2, true);
                    performScoreAction();
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

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
        intakeLeft = hardwareMap.get(DcMotor.class, "il");
        intakeRight = hardwareMap.get(DcMotor.class, "ir");
        shooterRight = hardwareMap.get(DcMotor.class, "sr");
        shooterLeft = hardwareMap.get(DcMotor.class, "sl");


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



    private void performScoreAction() {
        // Wind-up shooter motors before firing
        shooterLeft.setPower(1);
        shooterRight.setPower(1);

        actionTimer.resetTimer();
        while (actionTimer.getElapsedTimeSeconds() < 1.5) {
            // Wait for motors to spin up
        }

        // Activate intake to shoot
        intakeLeft.setPower(-0.5);
        intakeRight.setPower(-0.5);

        actionTimer.resetTimer();
        while (actionTimer.getElapsedTimeSeconds() < 1.5) {
            // Shooting period
        }

        shooterLeft.setPower(0);
        shooterRight.setPower(0);

    }

    private void performPickupActionDuringPath() {
        // Keep intake running while follower is busy
        if (follower.isBusy()) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        } else {
            stopIntake();
        }
    }


    private void runIntakeDuringPickup() {
        intakeLeft.setPower(-1);
        intakeRight.setPower(-1);
    }

    private void stopIntake() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }




}



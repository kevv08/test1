package org.firstinspires.ftc.teamcode.java.CodeVault.auto.randomAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.java.CodeVault.auto.Constants;

@Autonomous(name = "testAuto2", group = "Examples")
public class testAuto2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private boolean isPerformingAction = false;

    private DcMotor intakeLeft, intakeRight;
    private DcMotor shooterLeft, shooterRight;

    private Path scorePreload;
    private final Pose startPose = new Pose(22.123, 124, Math.toRadians(144));
    private final Pose scorePose = new Pose(51.8, 94.7, Math.toRadians(135));
    private final Pose prePickUpPose1 = new Pose(52.3, 33, Math.toRadians(180));
    private final Pose pickUpPose1 = new Pose(14.8, 33, Math.toRadians(180));
    private final Pose prePickUpPose2 = new Pose(51.8, 57.9, Math.toRadians(180));
    private final Pose pickUpPose2 = new Pose(14, 58, Math.toRadians(180));

    private PathChain goToPickup1, grabPickup1, scorePickup1, goToPickup2, grabPickup2, scorePickup2;

    // Action state tracking
    private enum ActionState {IDLE, SPINNING_UP_SHOOTER, SHOOTING, INTAKING}
    private ActionState actionState = ActionState.IDLE;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickUpPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickUpPose1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prePickUpPose1, pickUpPose1))
                .setLinearHeadingInterpolation(prePickUpPose1.getHeading(), pickUpPose1.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose1, scorePose))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), scorePose.getHeading())
                .build();

        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickUpPose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickUpPose2.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(prePickUpPose2, pickUpPose2))
                .setLinearHeadingInterpolation(prePickUpPose2.getHeading(), pickUpPose2.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose2, scorePose))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Score preload
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                // Wait to reach score position, then shoot
                if (!follower.isBusy() && !isPerformingAction) {
                    startScoreAction();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for shooting to complete, then go to pickup 1
                if (!isPerformingAction) {
                    follower.followPath(goToPickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Reached pre-pickup, now grab pickup 1 with intake
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    startIntakeAction();
                    setPathState(4);
                }
                break;

            case 4:
                // Grabbed pickup 1, now score it
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;

            case 5:
                // At score position, shoot pickup 1
                if (!follower.isBusy() && !isPerformingAction) {
                    startScoreAction();
                    setPathState(6);
                }
                break;

            case 6:
                // Shooting complete, go to pickup 2
                if (!isPerformingAction) {
                    follower.followPath(goToPickup2, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Reached pre-pickup 2, now grab it
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    startIntakeAction();
                    setPathState(8);
                }
                break;

            case 8:
                // Grabbed pickup 2, now score it
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;

            case 9:
                // At score position, shoot pickup 2
                if (!follower.isBusy() && !isPerformingAction) {
                    startScoreAction();
                    setPathState(10);
                }
                break;

            case 10:
                // All done
                if (!isPerformingAction) {
                    setPathState(-1);
                }
                break;
        }
    }

    // Non-blocking action updates (called every loop)
    public void updateActions() {
        switch (actionState) {
            case SPINNING_UP_SHOOTER:
                if (actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    // Spin-up complete, start shooting
                    intakeLeft.setPower(-0.5);
                    intakeRight.setPower(-0.5);
                    actionState = ActionState.SHOOTING;
                    actionTimer.resetTimer();
                }
                break;

            case SHOOTING:
                if (actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    // Shooting complete, stop everything
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    actionState = ActionState.IDLE;
                    isPerformingAction = false;
                }
                break;

            case INTAKING:
                // Intake continues until explicitly stopped
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }

    private void startScoreAction() {
        isPerformingAction = true;
        actionState = ActionState.SPINNING_UP_SHOOTER;
        shooterLeft.setPower(1);
        shooterRight.setPower(1);
        actionTimer.resetTimer();
    }

    private void startIntakeAction() {
        actionState = ActionState.INTAKING;
        intakeLeft.setPower(-1);
        intakeRight.setPower(-1);
    }

    private void stopIntake() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        if (actionState == ActionState.INTAKING) {
            actionState = ActionState.IDLE;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

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
        updateActions(); // Non-blocking action updates

        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionState);
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
}
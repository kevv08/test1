package org.firstinspires.ftc.teamcode.java.CodeVault.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.java.CodeVault.helper.Mechanum;


@TeleOp(name = "blueMeet0Bot", group = "Competition")
public class Meet0 extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {
        boolean upchargeOn = false;
        // --- Drive motors ---
      Mechanum drive = new Mechanum(hardwareMap,"fl","fr","bl","br");

        // --- Mechanisms ---
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake"); // EH 0
        DcMotor upchargeLeft = hardwareMap.get(DcMotor.class, "upLeft"); // 0
        DcMotor upchargeRight = hardwareMap.get(DcMotor.class, "upRight"); // EH 1

        CRServo rightIndexServo = hardwareMap.get(CRServo.class, "rightIndex"); // 0
        CRServo leftIndexServo = hardwareMap.get(CRServo.class, "leftIndex"); // EH 5

        // --- IMU setup ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // --- Motor configuration ---

        upchargeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIndexServo.setDirection(CRServo.Direction.REVERSE);




        DcMotor[] mechMotors = {intake, upchargeLeft, upchargeRight};
        for (DcMotor m : mechMotors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }



        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Reset IMU yaw
            if (gamepad1.back) imu.resetYaw();

            // --- Field-centric drive math ---
            double y = gamepad1.left_stick_y; // Forward/backward (reversed for correct direction)
            double x = -gamepad1.left_stick_x; // Strafing with slight adjustment
            double rx = -gamepad1.right_stick_x; // Rotation
            drive.drive(x, y, rx);


            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(1.0);
            }

            else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.left_bumper) {
                upchargeOn = true;
            } else if (gamepad1.a) {
                upchargeOn = false;
            }

            // --- apply motor power ---
            if (upchargeOn) {
                upchargeLeft.setPower(0.30);
                upchargeRight.setPower(0.30);
            } else {
                upchargeLeft.setPower(0);
                upchargeRight.setPower(0);
            }

            // trye ir false
            if (gamepad1.right_bumper) {
                rightIndexServo.setPower(1.0);
                leftIndexServo.setPower(1.0);
            } else {
                rightIndexServo.setPower(0);
                leftIndexServo.setPower(0);
            }

            // --- Telemetry ---

        }
    }
}

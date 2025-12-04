package org.firstinspires.ftc.teamcode.java.CodeVault.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.CodeVault.helper.fieldCentric;
import org.firstinspires.ftc.teamcode.java.CodeVault.helper.imuCall;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "blueMeet1Bot", group = "Competition")
public class Meet1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialize drivetrain class
        fieldCentric drive = new fieldCentric(hardwareMap, "fl", "fr", "bl", "br");

        // --- Initialize Shooter class
        Shooter shooter = new Shooter(hardwareMap, "sr", "sl");

        // --- Other motors and servos
        DcMotor intakeLeft  = hardwareMap.get(DcMotor.class, "il");
        DcMotor intakeRight = hardwareMap.get(DcMotor.class, "ir");

        Servo upchargeLeft  = hardwareMap.get(Servo.class, "upLeft");
        Servo upchargeRight = hardwareMap.get(Servo.class, "upRight");

        // --- Initialize IMU
        IMU imu = imuCall.initIMU(hardwareMap, "imu");

        // --- Variable for re-centering field orientation
        double headingOffset = 0;

        waitForStart();

        while (opModeIsActive()) {
            // Reset field-centric reference
            if (gamepad1.back) {
                headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // Get adjusted heading
            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = rawHeading - headingOffset;

            // Gamepad input
            double y  =  gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            drive.drive(x, y, rx, botHeading);

            // --- Intake controls
            if (gamepad1.left_bumper) {
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }

            // --- Shooter controls (handled by Shooter class)
            shooter.update(gamepad1.right_trigger);
            boolean shooterReady = shooter.isReady();   // <--- use this

            // --- Upcharge + reverse intake (only feed when ready)
            if (gamepad1.right_bumper && shooterReady) {
                upchargeLeft.setPosition(0.25);
                upchargeRight.setPosition(0.0);
                intakeLeft.setPower(-0.5);
                intakeRight.setPower(-0.5);
            } else {
                upchargeLeft.setPosition(0.0);
                upchargeRight.setPosition(0.25);
            }

            // --- Telemetry
            telemetry.addData("Heading (°)", Math.toDegrees(botHeading));
            telemetry.addData("Offset (°)", Math.toDegrees(headingOffset));
            shooter.addTelemetry(telemetry);  // shooter-specific info

            telemetry.update();
        }
    }
}

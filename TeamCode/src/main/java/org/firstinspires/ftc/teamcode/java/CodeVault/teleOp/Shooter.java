package org.firstinspires.ftc.teamcode.java.CodeVault.teleOp;  // <- change if you put it somewhere else

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    // You can tune these
    private static final double MAX_SHOOTER_VELOCITY = 1800; // ticks/sec
    private static final double SHOOTER_TOLERANCE    = 100;  // +/- ticks/sec
    private final DcMotorEx shooterRight;
    private final DcMotorEx shooterLeft;
    private double targetVelocity = 0;
    private double avgVelocity    = 0;
    private boolean shooterReady  = false;
    public Shooter(HardwareMap hardwareMap, String rightName, String leftName) {
        shooterRight = hardwareMap.get(DcMotorEx.class, rightName);
        shooterLeft  = hardwareMap.get(DcMotorEx.class, leftName);
        // Reverse one side if needed
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Use encoders for velocity control
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update(double triggerValue) {
        double shooterInput = triggerValue - 0.25;
        if (shooterInput < 0) shooterInput = 0; //deadzone
        targetVelocity = shooterInput * MAX_SHOOTER_VELOCITY;
        if (targetVelocity > 0) {
            shooterRight.setVelocity(targetVelocity);
            shooterLeft.setVelocity(targetVelocity);
        } else {
            shooterRight.setVelocity(0);
            shooterLeft.setVelocity(0);
        }
        double velRight = shooterRight.getVelocity();
        double velLeft  = shooterLeft.getVelocity();
        avgVelocity = (velRight + velLeft) / 2.0;
        shooterReady = targetVelocity > 0 &&
                Math.abs(avgVelocity - targetVelocity) < SHOOTER_TOLERANCE;
    }
    public boolean isReady() {
        return shooterReady;
    }
    public double getTargetVelocity() {
        return targetVelocity;
    }
    public double getAvgVelocity() {
        return avgVelocity;
    }
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter target vel", targetVelocity);
        telemetry.addData("Shooter avg vel",    avgVelocity);
        telemetry.addData("Shooter state", shooterReady ? "READY" : "spinning...");
    }
}

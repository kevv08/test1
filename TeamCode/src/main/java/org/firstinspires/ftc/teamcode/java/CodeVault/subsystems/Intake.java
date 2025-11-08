package org.firstinspires.ftc.teamcode.java.CodeVault.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.core.subsystems.Subsystem;


public class Intake implements Subsystem {
    // put hardware, commands, etc here
    DcMotor intakeLeft  = hardwareMap.get(DcMotor.class, "il");
    DcMotor intakeRight = hardwareMap.get(DcMotor.class, "ir");



    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {
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

    }
}
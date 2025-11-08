package org.firstinspires.ftc.teamcode.java.CodeVault.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

public class Gate implements Subsystem {
    // put hardware, commands, etc here
    Servo upchargeLeft  = hardwareMap.get(Servo.class, "upLeft");
    Servo upchargeRight = hardwareMap.get(Servo.class, "upRight");

    @Override
    public void initialize() {
        // initialization logic (runs on init)
       // public Command openClaw = new SetPosition(upchargeLeft, 1).requires(upchargeLeft);
     //   public Command closeClaw = new SetPosition(upchargeRight, 0).requires(upchargeRight);


    }

    @Override
    public void periodic() {
        if (gamepad1.right_bumper) {
            upchargeLeft.setPosition(0.25); // up
            upchargeRight.setPosition(0.0); // up
        } else {
            upchargeLeft.setPosition(0.0);  // down
            upchargeRight.setPosition(0.25); // down
        }
    }
}
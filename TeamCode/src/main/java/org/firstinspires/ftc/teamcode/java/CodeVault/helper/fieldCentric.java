package org.firstinspires.ftc.teamcode.java.CodeVault.helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class fieldCentric {
    private final DcMotor fl, fr, bl, br;

    public fieldCentric(HardwareMap hw, String flName, String frName, String blName, String brName) {
        fl = hw.get(DcMotor.class, flName);
        fr = hw.get(DcMotor.class, frName);
        bl = hw.get(DcMotor.class, blName);
        br = hw.get(DcMotor.class, brName);

        // Example: reverse the right side if your build needs it
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    /** Drive field-centric. headingRad is robot yaw in radians. */
    public void drive(double x, double y, double rx, double botHeading) {
        // rotate joystick vector by -heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


        rotX *= 1.1; // compensate for imperfect strafing (optional)

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPow = (rotY +rotX + rx) / denominator;
        double blPow = (rotY - rotX + rx) / denominator;
        double frPow= (rotY -rotX - rx) / denominator;
        double brPow = (rotY + rotX - rx) / denominator;



        fl.setPower(flPow);
        bl.setPower(blPow);
        fr.setPower(frPow);
        br.setPower(brPow);
        if (flPow == 0 && blPow == 0 && frPow == 0 && brPow == 0) {
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


    }
}




// Retrieve the IMU from the hardware map
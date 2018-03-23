package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 3/22/2018.
 */
//DO NOT MODIFY THIS! USE ONLY FOR TANK DRIVE TRAIN!
@TeleOp (name = "TankDrive", group = "")
public class TankDrive extends OpMode
{
    DcMotor motorLF;
    DcMotor motorRF;
    DcMotor motorLB;
    DcMotor motorRB;

    public void init()
    {
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        motorRB = hardwareMap.dcMotor.get("motorRB");

        motorRB.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorLF.setPower(0);

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop()
    {
        motorRB.setPower(-gamepad1.right_stick_y);
        motorRF.setPower(-gamepad1.right_stick_y);
        motorLF.setPower(gamepad1.left_stick_y);
        motorLB.setPower(gamepad1.left_stick_y);
    }
}

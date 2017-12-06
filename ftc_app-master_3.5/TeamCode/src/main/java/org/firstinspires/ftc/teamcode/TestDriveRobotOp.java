package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by User on 12/5/2017.
 */
@TeleOp(name = "ConnerTestDrive", group = "Testing")
public class TestDriveRobotOp extends LinearOpMode
{
    double speed = 1;
    double zSclae = 0.75;

    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;




    @Override
    public void runOpMode()
    {
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        waitForStart();

        while (opModeIsActive())
        {
            motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zSclae * gamepad1.right_stick_x)));
            motorLB.setPower(speed*(-(-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zSclae * gamepad1.right_stick_x)));
            motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zSclae * gamepad1.right_stick_x)));
            motorLF.setPower(speed*(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zSclae * gamepad1.right_stick_x));
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 11/4/2017.
 */

@TeleOp(name = "TeleOpTesting", group = "")
public class TeleOpTesting extends LinearOpMode
{

    DcMotor joint2;
    DcMotor joint3;
    double joint1Speed = 0.50;
    double joint2Speed = 0.50;
    double joint3MaxSpeed = 1.0;

    public void runOpMode()
    {
        joint2 = hardwareMap.dcMotor.get("joint2");
        joint3 = hardwareMap.dcMotor.get("joint3");
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive())
        {
            joint3.setPower((gamepad2.left_stick_y * joint3MaxSpeed));

            //Setting Joint 2
            if(gamepad2.left_bumper)
            {
                joint2.setPower(joint2Speed);
            }
            else if(gamepad2.left_trigger > 0.4)
            {
                joint2.setPower(-joint2Speed);
            }
            else
            {
                joint2.setPower(0.0);
            }

            telemetry.addData("Joint3 Speed", joint3.getPower());
            telemetry.addData("Joint2 Speed", joint2.getPower());
            telemetry.update();
        }
    }
}

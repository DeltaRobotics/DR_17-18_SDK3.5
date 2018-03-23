package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 3/22/2018.
 */

@TeleOp (name = "SummerTesting", group = "")
public class SummerTesting extends LinearOpMode
{
    DcMotor arm;

    int holdPosition = 0;
    double armScaling = 0.5;
    boolean runOnce = true;
    boolean runOnce2 = true;

    @Override
    public void runOpMode()
    {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setPower(0.0);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.left_stick_y < 0.1 && gamepad1.left_stick_y > -0.1)
            {
                runOnce2 = true;
                if(runOnce)
                {
                    holdPosition = arm.getCurrentPosition();
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm.setPower(1.0);
                    runOnce = false;
                }

                arm.setTargetPosition(holdPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1)
            {
                arm.setPower(gamepad1.left_stick_y * armScaling);
                runOnce = true;
                if(runOnce2)
                {
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    runOnce2 = false;
                }
            }

            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("HoldPosition", holdPosition);
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Left Joystick Y", gamepad1.left_stick_y);
            telemetry.addData("Arm Mode", arm.getMode());
            telemetry.update();
        }
    }
}

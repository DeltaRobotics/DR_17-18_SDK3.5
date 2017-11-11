package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 10/5/2017.
 */

@TeleOp (name = "MotorTest", group = "")
public class MotorTest extends LinearOpMode
{
    DcMotor motor;
    double motorPower = 0.5;
    boolean runMotors = false;

    boolean leftDPadState = false;
    boolean rightDPadState = false;

    @Override
    public void runOpMode()
    {
        motor = hardwareMap.dcMotor.get("motor");

        waitForStart();
        while(opModeIsActive())
        {
            if (gamepad1.a)
            {
                runMotors = true;
            }

            if (gamepad1.b)
            {
                runMotors = false;
            }

            if (gamepad1.dpad_left && !leftDPadState && runMotors)
            {
                leftDPadState = true;
                motorPower -= 0.05;
            } else if (!gamepad1.dpad_left)
            {
                leftDPadState = false;
            }

            if (gamepad1.dpad_right && !rightDPadState && runMotors)
            {
                rightDPadState = true;
                motorPower += 0.05;
            } else if (!gamepad1.dpad_right)
            {
                rightDPadState = false;
            }

            if (motorPower > 1.0)
            {
                motorPower = 1.0;
            }

            if (motorPower < -1.0)
            {
                motorPower = -1.0;
            }

            if(runMotors)
            {
                motor.setPower(motorPower);
            }
            else
            {
                motor.setPower(0.0);
            }

            telemetry.addData("motor Power", motor.getPower());
            telemetry.addData("motorPower Var", motorPower);
            telemetry.update();
        }
    }
}

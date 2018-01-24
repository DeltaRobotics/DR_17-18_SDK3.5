package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by User on 1/23/2018.
 */

@TeleOp (name="SpeedTest", group = "")
public class SpeedTest extends LinearOpMode
{
    RobotHardware curiosity = new RobotHardware();
    Drive drive = new Drive();

    double motorPower = 0.5;

    boolean dPadUpState = false;
    boolean dPadDownState = false;

    @Override
    public void runOpMode()
    {
        curiosity.init(hardwareMap);
        curiosity.slapper.setPosition(0.8);
        DcMotor[] motors = new DcMotor[4];
        motors[0] = curiosity.motorRF;
        motors[1] = curiosity.motorRB;
        motors[2] = curiosity.motorLB;
        motors[3] = curiosity.motorLF;
        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.y)
            {
                drive.encoderDrive(1000, driveStyle.FORWARD, motorPower, motors);
            }

            if(gamepad1.a)
            {
                drive.encoderDrive(1000, driveStyle.BACKWARD, motorPower, motors);
            }

            if(gamepad1.x)
            {
                drive.encoderDrive(1000, driveStyle.STRAFE_LEFT, motorPower, motors);
            }

            if(gamepad1.b)
            {
                drive.encoderDrive(1000, driveStyle.STRAFE_RIGHT, motorPower, motors);
            }

            if(gamepad1.left_bumper)
            {
                drive.encoderDrive(1000, driveStyle.PIVOT_LEFT, motorPower, motors);
            }

            if(gamepad1.right_bumper)
            {
                drive.encoderDrive(1000, driveStyle.PIVOT_RIGHT, motorPower, motors);
            }

            if(!dPadUpState && gamepad1.dpad_up)
            {
                dPadUpState = true;
                motorPower += 0.05;
            }
            else if(!gamepad1.dpad_up)
            {
                dPadUpState = false;
            }

            if(!dPadDownState && gamepad1.dpad_down)
            {
                dPadDownState = true;
                motorPower -= 0.05;
            }
            else if(!gamepad1.dpad_down)
            {
                dPadDownState = false;
            }

            motorPower = Range.clip(motorPower, 0, 1);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();
        }
    }
}

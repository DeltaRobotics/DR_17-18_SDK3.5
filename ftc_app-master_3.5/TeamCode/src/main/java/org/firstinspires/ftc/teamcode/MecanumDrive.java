package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 9/16/2017.
 */

@TeleOp (name = "Mecanum Drive", group = "")
public class MecanumDrive extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive();

    boolean bState = false;
    boolean xState = false;
    boolean yState = false;
    boolean aState = false;

    public void runOpMode()
    {
        robot.init(hardwareMap);

        robot.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.flapper.setPosition(1.0);
        robot.slapper.setPosition(0.35);

        waitForStart();

        while (opModeIsActive())
        {


            /*if (gamepad1.right_stick_x == 0)
            {
                motorRF.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x));
                motorLB.setPower(-(-gamepad1.left_stick_y - gamepad1.left_stick_x));
                motorRB.setPower(-(-gamepad1.left_stick_x + gamepad1.left_stick_y));
                motorLF.setPower((-gamepad1.left_stick_x + gamepad1.left_stick_y));
            }


            if (gamepad1.right_stick_x != 0)
            {
                motorLF.setPower(-gamepad1.right_stick_x);
                motorLB.setPower(-gamepad1.right_stick_x);
                motorRF.setPower(-gamepad1.right_stick_x);
                motorRB.setPower(-gamepad1.right_stick_x);
            }
            */


            robot.motorRF.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
            robot.motorLB.setPower(-(-gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
            robot.motorRB.setPower(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - gamepad1.right_stick_x);
            robot.motorLF.setPower((-gamepad1.left_stick_x + gamepad1.left_stick_y) - gamepad1.right_stick_x);





            if(gamepad1.y && !yState)
            {
                yState = true;
                robot.slapper.setPosition(robot.slapper.getPosition() + 0.05);
            }
            else if(!gamepad1.y)
            {
                yState = false;
            }

            if(gamepad1.a && !aState)
            {
                aState = true;
                robot.slapper.setPosition(robot.slapper.getPosition() - 0.05);
            }
            else if(!gamepad1.a)
            {
                aState = false;
            }

            if(gamepad1.x && !xState)
            {
                xState = true;
                if(robot.flapper.getPosition() >= 0.60 || robot.slapper.getPosition() == 1)
                {
                    robot.flapper.setPosition(robot.slapper.getPosition() - 0.05);
                }

            }
            else if(!gamepad1.x)
            {
                xState = false;
            }

            if(gamepad1.b && !bState)
            {
                bState = true;
                if(robot.flapper.getPosition() >= 0.55 || robot.slapper.getPosition() == 1)
                {
                    robot.flapper.setPosition(robot.flapper.getPosition() + 0.05);
                }
            }
            else if(!gamepad1.b)
            {
                bState = false;
            }

            telemetry.addData("Slapper Pos", robot.slapper.getPosition());
            telemetry.addData("Flapper Pos", robot.flapper.getPosition());

            telemetry.addData("Left Joystick X", gamepad1.left_stick_x);
            telemetry.addData("Left Joystick Y", gamepad1.left_stick_y);
            telemetry.addData("Right Joystick X", gamepad1.right_stick_x);
            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());

            telemetry.update();

            //Comment
//

        }
    }
}

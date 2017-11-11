package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.driveStyle;

/**
 * Created by User on 9/26/2017.
 */

@Autonomous (name = "AutoTesting", group = "")
public class AutoTesting extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive();

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

       /*
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

*/
        robot.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //


        waitForStart();

            /*robot.motorRB.setTargetPosition(2000);
            robot.motorLB.setTargetPosition(-2000);

            robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */


            /*robot.motorRF.setPower(-0.5);
            robot.motorLF.setPower(0.5);
            robot.motorRB.setPower(-0.5);
            robot.motorLB.setPower(0.5);

            sleep(1000);

        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorLB.setPower(0);

        sleep(500);

            robot.motorRF.setPower(-0.5);
            robot.motorLF.setPower(-0.5);
            robot.motorRB.setPower(0.5);
            robot.motorLB.setPower(0.5);

            sleep(1000);

        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorLB.setPower(0);

        sleep(500);

            robot.motorRF.setPower(0.5);
            robot.motorLF.setPower(-0.5);
            robot.motorRB.setPower(0.5);
            robot.motorLB.setPower(-0.5);

            sleep(1000);

        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorLB.setPower(0);

        sleep(500);

            robot.motorRF.setPower(0.5);
            robot.motorLF.setPower(0.5);
            robot.motorRB.setPower(-0.5);
            robot.motorLB.setPower(-0.5);

        sleep(1000);

        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorLB.setPower(0);

/*
            while(opModeIsActive() && robot.motorRB.isBusy() && robot.motorLB.isBusy())
            {
                telemetry.addData("motorRF Pos", robot.motorRF.getCurrentPosition());
                telemetry.addData("motorLF Pos", robot.motorLF.getCurrentPosition());
                telemetry.addData("motorRB Pos", robot.motorRB.getCurrentPosition());
                telemetry.addData("motorLB Pos", robot.motorLB.getCurrentPosition());
                telemetry.addData("motorRF Power", robot.motorRF.getPower());
                telemetry.addData("motorLF Power", robot.motorLF.getPower());
                telemetry.addData("motorRB Power", robot.motorRB.getPower());
                telemetry.addData("motorLB Power", robot.motorLB.getPower());

                telemetry.update();
            }

            robot.motorRF.setPower(0);
            robot.motorLF.setPower(0);
            robot.motorRB.setPower(0);
            robot.motorLB.setPower(0);


            robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            */

        /*
            sleep(3000);


            robot.motorRB.setTargetPosition(2000);
            robot.motorLB.setTargetPosition(-2000);

            robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motorRF.setPower(0.5);
            robot.motorLF.setPower(-0.5);
            robot.motorRB.setPower(0.5);
            robot.motorLB.setPower(-0.5);

        while(opModeIsActive() && robot.motorRB.isBusy() && robot.motorLB.isBusy())
        {
            telemetry.addData("motorRF Pos", robot.motorRF.getCurrentPosition());
            telemetry.addData("motorLF Pos", robot.motorLF.getCurrentPosition());
            telemetry.addData("motorRB Pos", robot.motorRB.getCurrentPosition());
            telemetry.addData("motorLB Pos", robot.motorLB.getCurrentPosition());
        }

/*
        robot.motorRF.setPower(drive.setPower(0, -0.5, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, -0.5, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, -0.5, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, -0.5, 0)[3]);

        sleep(30000);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        /*sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0.5, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0.5, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0.5, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0.5, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(-0.5, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(-0.5, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(-0.5, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(-0.5, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0.5, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0.5, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0.5, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0.5, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(-0.5, -0.5, 0)[0]);
        robot.motorRB.setPower(drive.setPower(-0.5, -0.5, 0)[1]);
        robot.motorLB.setPower(drive.setPower(-0.5, -0.5, 0)[2]);
        robot.motorLF.setPower(drive.setPower(-0.5, -0.5, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0.5, 0.5, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0.5, 0.5, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0.5, 0.5, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0.5, 0.5, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0.5, -0.5, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0.5, -0.5, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0.5, -0.5, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0.5, -0.5, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(-0.5, 0.5, 0)[0]);
        robot.motorRB.setPower(drive.setPower(-0.5, 0.5, 0)[1]);
        robot.motorLB.setPower(drive.setPower(-0.5, 0.5, 0)[2]);
        robot.motorLF.setPower(drive.setPower(-0.5, 0.5, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0.5)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0.5)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0.5)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0.5)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, -0.5)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, -0.5)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, -0.5)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, -0.5)[3]);

        sleep(500);

        robot.motorRF.setPower(drive.setPower(0, 0, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, 0, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, 0, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, 0, 0)[3]);

        sleep(500);
        */

        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;


        /*drive.timeDrive(1000, 0.5, driveStyle.FORWARD, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.BACKWARD, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.STRAFE_LEFT, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.STRAFE_RIGHT, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.FORWARD_LEFT, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.BACKWARD_RIGHT, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.FORWARD_RIGHT, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.BACKWARD_LEFT, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.PIVOT_LEFT, motors);
        sleep(500);
        drive.timeDrive(1000, 0.5, driveStyle.PIVOT_RIGHT, motors);
        */

        drive.encoderDrive(500, driveStyle.FORWARD, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.BACKWARD, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.STRAFE_LEFT, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.STRAFE_RIGHT, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.FORWARD_RIGHT, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.BACKWARD_LEFT, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.FORWARD_LEFT, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.BACKWARD_RIGHT, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.PIVOT_LEFT, 0.5, 5000, motors);
        sleep(3000);
        drive.encoderDrive(500, driveStyle.PIVOT_RIGHT, 0.5, 5000, motors);
        sleep(3000);

        /*robot.motorRF.setPower(drive.setPower(0, -0.5, 0)[0]);
        robot.motorRB.setPower(drive.setPower(0, -0.5, 0)[1]);
        robot.motorLB.setPower(drive.setPower(0, -0.5, 0)[2]);
        robot.motorLF.setPower(drive.setPower(0, -0.5, 0)[3]);
        */





    }
}

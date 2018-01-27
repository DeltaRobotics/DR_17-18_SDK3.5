package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 1/25/2018.
 */

@TeleOp(name = "MotorTesting", group = "Testing")
public class MotorTestingNew extends LinearOpMode
{
    DcMotor testMotor1;
    DcMotor testMotor2;
    boolean slow = true;
    public void runOpMode() throws InterruptedException
    {
        testMotor1 = hardwareMap.dcMotor.get("motor1");
        testMotor2 = hardwareMap.dcMotor.get("motor2");
        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive())
        {
            if(slow)
            {
                testMotor1.setPower(gamepad1.right_stick_y * 0.3);
                testMotor2.setPower(gamepad1.right_stick_y * 0.3);
            }
            if(!slow)
            {
                testMotor1.setPower(gamepad1.right_stick_y);
                testMotor2.setPower(gamepad1.right_stick_y);
            }
            if(gamepad1.b)
            {
                slow = true;
            }
            if(gamepad1.a)
            {
                slow = false;
            }
            telemetry.addData("Motor power", testMotor1.getPower());
            telemetry.addData("Slow Mode On?", slow);
            telemetry.update();
        }


    }
}

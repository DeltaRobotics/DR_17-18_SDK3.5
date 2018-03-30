package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by User on 3/29/2018.
 */
@TeleOp(name = "Voltage Testing", group = "Off-SeasonTesting")
public class Voltage_Testing extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();

    VoltageSensor motorLFvoltage;
    VoltageSensor motorRFvoltage;
    VoltageSensor motorLBvoltage;
    VoltageSensor motorRBvoltage;
    VoltageSensor robotVoltage;
    double LFvolts;
    double RFvolts;
    double LBvolts;
    double RBvolts;
    double robotVolts;

    double zSclae = 0.75;
    double speed = 1.0;

    double voltageMax = 0;
    double voltageMin = 20;
    double voltageNow = 0.0;

    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Robot Init", "Completed");
        telemetry.update();
        waitForStart();
        //Initializing the robot, and waiting for the start button to be pressed

        while (opModeIsActive())
        {
            robot.motorRF.setPower(speed*((-gamepad1.right_stick_y - gamepad1.right_stick_x) - (zSclae * gamepad1.left_stick_x)));
            robot.motorLB.setPower(speed*(-(-gamepad1.right_stick_y - gamepad1.right_stick_x) - (zSclae * gamepad1.left_stick_x)));
            robot.motorRB.setPower(speed*(-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (zSclae * gamepad1.left_stick_x)));
            robot.motorLF.setPower(speed*(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (zSclae * gamepad1.left_stick_x));
            //Linking joystick movement (gamepad1) to wheel motion and power

            robotVoltage = hardwareMap.voltageSensor.get("Expansion Hub 10");
            //accessing expansion hub voltage
            voltageNow = robotVoltage.getVoltage();
            //deciphering voltage from sensor
            if(voltageNow < voltageMin)
            {
                voltageMin = voltageNow;
            }
            //Finding minimum voltage

            if(voltageNow > voltageMax)
            {
                voltageMax = voltageNow;
            }
            //Finding maximum voltage
            telemetry.addData("Voltage", voltageNow);
            telemetry.addData("Minimum", voltageMin);
            telemetry.addData("Maximum", voltageMax);
            //Sending telemetry to the driver's station
            telemetry.update();
        }


    }
}

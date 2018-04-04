package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.security.Timestamp;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by User on 3/29/2018.
 */
@TeleOp(name = "Voltage Testing", group = "Off-SeasonTesting")
public class Voltage_Testing extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
    String root = Environment.getExternalStorageDirectory().toString();


    SimpleDateFormat s = new SimpleDateFormat("ddhhmmss");
    String format = s.format(new Date());
    String fname = format +".txt";

    File myDir = new File(root);

    FileOutputStream out = null;

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
    int counter =0;

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


        myDir.mkdir();

        File saveFile = new File(myDir, fname);

        if (saveFile.exists ()) saveFile.delete ();
        try
            {
                out = new FileOutputStream(saveFile);
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }



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
            //Finding minimum voltage of robot

            if(voltageNow > voltageMax)
            {
                voltageMax = voltageNow;
            }
            counter++;
            //Finding maximum voltage of robot
            telemetry.addData("Voltage", voltageNow);
            telemetry.addData("Minimum", voltageMin);
            telemetry.addData("Maximum", voltageMax);
            telemetry.addData("loop count", counter);

            //Sending telemetry to the driver's station
            telemetry.update();


            try {

                out.write((voltageNow + "\r\n").getBytes());
                //out.close();

            } catch (Exception e) {
                e.printStackTrace();
            }

        }

    }


}

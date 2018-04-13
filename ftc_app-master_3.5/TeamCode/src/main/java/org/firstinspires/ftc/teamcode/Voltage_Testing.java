package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.security.Timestamp;
import java.text.SimpleDateFormat;
import java.util.Date;

import static org.firstinspires.ftc.teamcode.Voltage_Testing.addIndex;
import static org.firstinspires.ftc.teamcode.Voltage_Testing.outIndex;

/**
 * Created by User on 3/29/2018.
 */
@TeleOp(name = "Voltage Testing", group = "Off-SeasonTesting")
public class Voltage_Testing extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
    ElapsedTime runtime = new ElapsedTime();
    //String root = Environment.getExternalStorageDirectory().toString();
    long beginTime;
    long loopTime;
    boolean first;

    boolean stall = false;

    int loopCount = 0;

    double targetRuntime = 0.0;


    /*SimpleDateFormat s = new SimpleDateFormat("ddhhmmss");
    String format = s.format(new Date());
    String fname = "VoltageData" + format +".txt";

    File myDir = new File(root);

    FileOutputStream out = null;
    */

    VoltageSensor robotVoltage;

    String[] voltageArr = new String[100000];
    String newLine;
    static int addIndex = 0;
    static int outIndex = 0;

    double robotVolts;

    double zSclae = 0.75;
    double speed = 1.0;

    double voltageMax = 0;
    double voltageMin = 20;
    double voltageNow = 0.0;

    double startVoltage = 0.0;

    double stallVoltage = 0.0;

    public void runOpMode()
    {
        VoltageThread v = new VoltageThread(voltageArr);
        v.start();
        robot.init(hardwareMap);
        telemetry.addData("Robot Init", "Completed");
        telemetry.update();

        /*String root = Environment.getExternalStorageDirectory().toString();
        File myDir = new File(root);

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
            */

        waitForStart();
        //Initializing the robot, and waiting for the start button to be pressed

        while (opModeIsActive())
        {
            loopCount++;

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
            //Finding maximum voltage of robot

            if(first)
            {
                first = false;
                beginTime = System.currentTimeMillis();
            }
            loopTime = System.currentTimeMillis() - beginTime;
            newLine = Long.toString(loopTime) + " " + roundNumstoString(voltageNow) + " " + roundNumstoString(robot.motorLF.getPower()) + " " + roundNumstoString(robot.motorRF.getPower()) +" " + roundNumstoString(robot.motorLB.getPower()) + " " + roundNumstoString(robot.motorRB.getPower()) + "\r\n";

            voltageArr[addIndex] = newLine;
            /*try {

                out.write((loopTime + " " + ((float)Math.round(voltageNow * 1000) / 1000) + " " + ((float)Math.round(robot.motorLF.getPower() * 1000) / 1000) + " " + ((float)Math.round(robot.motorRF.getPower() * 1000) / 1000) +" " + ((float)Math.round(robot.motorLB.getPower() * 1000) / 1000) + " " + ((float)Math.round(robot.motorRB.getPower() * 1000) / 1000) +"\r\n").getBytes());
                //out.close();

            } catch (Exception e) {
                e.printStackTrace();
            }*/
            if(loopCount == 3)
            {
                startVoltage = voltageNow;
                stallVoltage = startVoltage - 2.0;

            }

            if(voltageNow <= stallVoltage)
            {
                if(targetRuntime == 0)
                {
                    targetRuntime = runtime.milliseconds() + 750;
                }
                if(runtime.milliseconds() >= targetRuntime)
                {
                    stall = true;
                }
            }
            else
            {
                targetRuntime = 0;
            }
            addIndex++;
            telemetry.addData("Voltage", voltageNow);
            telemetry.addData("Minimum", voltageMin);
            telemetry.addData("Maximum", voltageMax);
            telemetry.addData("addIndex", addIndex);
            telemetry.addData("outIndex", outIndex);
            telemetry.addData("Data?", voltageArr[addIndex]);
            telemetry.addData("Start Voltage", startVoltage);
            telemetry.addData("Target Runtime", targetRuntime);
            telemetry.addData("Stall", stall);

            //Sending telemetry to the driver's station
            telemetry.update();

            if(addIndex > 100000)
            {
                v.interrupt();
            }


        }
    }
    public String roundNumstoString(double num)
    {
        String returnNum;
        returnNum = Double.toString((float)Math.round(num * 1000) / 1000);

        return returnNum;
    }
}

class VoltageThread extends Thread
{
    String root = Environment.getExternalStorageDirectory().toString();

    SimpleDateFormat s = new SimpleDateFormat("ddhhmmss");
    String format = s.format(new Date());
    String fname = "VoltageData" + format +".txt";

    File myDir = new File(root);

    FileOutputStream out = null;


    String[] voltageLine;
    VoltageThread(String[] voltageLine)
    {
        this.voltageLine = voltageLine;
    }

    public void run()
    {
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

        while(addIndex < 100000){
            if((addIndex != 0) && (addIndex > outIndex))
            {
                try {

                    out.write((outIndex + " " + voltageLine[outIndex]).getBytes());
                    outIndex++;
                    //out.close();

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }

    }
}

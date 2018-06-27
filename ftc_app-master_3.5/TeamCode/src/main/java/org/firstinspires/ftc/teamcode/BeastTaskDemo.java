package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import static org.firstinspires.ftc.teamcode.Voltage_Testing.addIndex;
import static org.firstinspires.ftc.teamcode.Voltage_Testing.outIndex;

/**
 * Created by User on 6/5/2018.
 */



public class BeastTaskDemo extends LinearOpMode
{
    RobotHardware robot = new RobotHardware();
    VoltageSensor robotVoltage;

    boolean allTasksCompleted = false;
    boolean taskFinished = true;//this is set to true so it will on the first iteration set taskIndex to 0 and set currentTask
    int taskIndex = -1;//tell current task what task it is on
    int paramIndex = 0;
    int motorRFPos = 0;//RF = Right Front
    int motorLFPos = 0;//LF = Left Front
    int motorRBPos = 0;//RB = Right Back
    int motorLBPos = 0;//LB = Left Back


    String[] currentTask = null;//pulls corisponding task from the array
    String[] methodParams = new String[100];
    String[] voltageArr = new String[100000];
    String[][] tasks = new String[100][100];

    //File logfile = File(new);
    Thread telemetryThread = new Thread();

    double voltageNow = 0.0;



    public void runOpMode()
    {
        tasks[0][0] = "drive";//Function
        tasks[0][1] = "drive forward 50% power 1000 encoder counts";//Description
        tasks[0][2] = "0.5";//Parameter 0
        tasks[0][3] = "forward";//Parameter 1
        tasks[0][4] = "1000";//Parameter 2

        tasks[1][0] = "drive";
        tasks[1][1] = "drive backward 50% power 1000 encoder counts";
        tasks[1][2] = "0.5";
        tasks[1][3] = "backward";
        tasks[1][4] = "1000";

        robot.init(hardwareMap);
        VoltageFileThread v = new VoltageFileThread(voltageArr);
        v.start();
        waitForStart();

        while(opModeIsActive() && !allTasksCompleted)
        {
            if(taskFinished)
            {
                taskIndex++;//increments taskIndex
                taskFinished = false;//makes it so in the next loop it will not increment task index at the wrong time
                currentTask = tasks[taskIndex];

            }

            //Read Robot Status
            motorRFPos = robot.motorRF.getCurrentPosition();//getting motor encoder count and updating its variable
            motorLFPos = robot.motorLF.getCurrentPosition();
            motorRBPos = robot.motorRB.getCurrentPosition();
            motorLBPos = robot.motorLB.getCurrentPosition();
            //telemetry.addData();
            //updateTelemetry();
            methodParams = currentTask;
        }
    }
}

class VoltageFileThread extends Thread
{
    String root = Environment.getExternalStorageDirectory().toString();

    SimpleDateFormat s = new SimpleDateFormat("ddhhmmss");
    String format = s.format(new Date());
    String fname = "VoltageData" + format +".txt";

    File myDir = new File(root);

    FileOutputStream out = null;


    String[] voltageLine;
    VoltageFileThread(String[] voltageLine)
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
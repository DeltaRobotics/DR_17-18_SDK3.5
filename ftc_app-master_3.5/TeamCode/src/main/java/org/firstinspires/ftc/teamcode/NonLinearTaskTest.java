package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by User on 5/1/2018.
 */

public class NonLinearTaskTest extends OpMode
{


    enum Tasks
    {
        DRIVE, MANIP, SENSOR
    }

    RobotHardware robot = new RobotHardware();

    Tasks tasks = Tasks.DRIVE;
    public Tasks[] taskArray = new Tasks[10];

    public void init()
    {
        robot.init(hardwareMap);
        taskArray[0] = tasks.DRIVE;
        taskArray[1] = tasks.MANIP;
        taskArray[2] = tasks.SENSOR;
    }

    public void loop()
    {
        switch (tasks)
        {
            case DRIVE:
            {
              break;
            }

            case MANIP:
            {
                break;
            }

            case SENSOR:
            {
                break;
            }

        }
    }
}

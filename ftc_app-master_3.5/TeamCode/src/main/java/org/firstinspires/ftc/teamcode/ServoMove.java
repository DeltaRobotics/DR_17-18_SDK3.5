package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by User on 10/28/2017.
 */

public class ServoMove extends LinearOpMode
{
    public void knockOffJewel(Servo[] servos, int color, String alliance)
    {
        sleep(2000);
        servos[1].setPosition(0.3);
        sleep(1000);
        servos[0].setPosition(0.68);
        sleep(1000);
        if(color == 0 && alliance.equals("blue"))
        {
            servos[1].setPosition(0.10);
            sleep(1000);
            servos[1].setPosition(0.15);
        }
        if(color == 0 && alliance.equals("red"))
        {
            servos[1].setPosition(0.50);
            sleep(1000);
            servos[1].setPosition(0.40);
        }
        if(color == 1 && alliance.equals("blue"))
        {
            servos[1].setPosition(0.50);
            sleep(1000);
            servos[1].setPosition(0.40);
        }
        if(color == 1 && alliance.equals("red"))
        {
            servos[1].setPosition(0.10);
            sleep(1000);
            servos[1].setPosition(0.15);
        }

        if(color == 2)
        {
            servos[1].setPosition(0.15);
        }

        sleep(1000);
        servos[0].setPosition(1.0);
        sleep(1000);
        servos[1].setPosition(0.8);
        sleep(1000);
        return ;
    }

    @Override
    public void runOpMode(){

    }
}

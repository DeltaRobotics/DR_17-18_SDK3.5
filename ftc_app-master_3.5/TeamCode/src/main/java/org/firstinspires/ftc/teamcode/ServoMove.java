package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        servos[0].setPosition(0.30);
        sleep(1000);
        if(color == 0 && alliance.equals("blue"))
        {
            servos[1].setPosition(0.1);
            sleep(1000);
            servos[1].setPosition(0.20);
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
            servos[1].setPosition(0.20);
        }

        if(color == 2)
        {
            servos[1].setPosition(0.20);
        }

        sleep(1000);
        servos[0].setPosition(0.7);
        sleep(1000);
        servos[1].setPosition(0.8);
        sleep(1000);
        return ;
    }

    public void placeGlyph(Servo[] servos, RobotHardware curiosity, Drive drive)
    {
        DcMotor[] motors = new DcMotor[4];
        motors[0] = curiosity.motorRF;
        motors[1] = curiosity.motorRB;
        motors[2] = curiosity.motorLB;
        motors[3] = curiosity.motorLF;
        sleep(250);
        servos[2].setPosition(0.15);
        sleep(750);
        servos[3].setPosition(0.85);
        sleep(750);
        drive.encoderDrive(100, driveStyle.BACKWARD, 0.5, motors);
        sleep(250);
        servos[2].setPosition(0.395);
        sleep(250);
        servos[3].setPosition(0.94);
        servos[2].setPosition(0.75);
        drive.encoderDrive(550, driveStyle.FORWARD, 0.5, motors);
        sleep(250);
        drive.encoderDrive(200, driveStyle.BACKWARD, 0.5, motors);
        //
    }

    @Override
    public void runOpMode(){

    }
}

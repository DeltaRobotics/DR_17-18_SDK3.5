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
        /*servos[1].setPosition(0.3); // rotates slapper
        sleep(500);*/
        servos[0].setPosition(0.30); //arm down
        sleep(1500);
        if(color == 0 && alliance.equals("blue"))
        {
            servos[1].setPosition(0.1); //Knocks off jewl
            sleep(750);
            servos[1].setPosition(0.20); //Moves slapper back
        }
        if(color == 0 && alliance.equals("red"))
        {
            servos[1].setPosition(0.50); //Knocks off jewl
            sleep(750);
            servos[1].setPosition(0.40); //Moves slapper back
        }
        if(color == 1 && alliance.equals("blue"))
        {
            servos[1].setPosition(0.50); //Knocks off jewl
            sleep(750);
            servos[1].setPosition(0.40); //Moves slapper back
        }
        if(color == 1 && alliance.equals("red"))
        {
            servos[1].setPosition(0.10); //Knocks off jewl
            sleep(750);
            servos[1].setPosition(0.20); //Moves slapper back
        }

        if(color == 2)
        {
            servos[1].setPosition(0.20); //Slapper stays mid
        }

        sleep(750);
        servos[0].setPosition(0.6); //Raises flapper up
        sleep(750);
        servos[1].setPosition(0.8); //Moves slapper to home position
        sleep(750);
        return ;
    }

    public void placeGlyph(Servo[] servos, RobotHardware curiosity, Drive drive)
    {
        DcMotor[] motors = new DcMotor[4];//]
        motors[0] = curiosity.motorRF; //]
        motors[1] = curiosity.motorRB; //]Drive train motor array
        motors[2] = curiosity.motorLB; //]
        motors[3] = curiosity.motorLF; //]
        sleep(250);
        servos[2].setPosition(0.05); //Moves knock out
        sleep(750);
        servos[3].setPosition(0.80); //Opens claw
        sleep(750);
        drive.encoderDrive(100, driveStyle.BACKWARD, 0.5, motors); //Moves robot back
        sleep(250);
        servos[2].setPosition(0.395); //Moves knock to a mid position
        sleep(250);
        servos[3].setPosition(0.94); //Closes claw
        servos[2].setPosition(0.75); //Moves knock to home position
        drive.encoderDrive(650, driveStyle.FORWARD, 0.5, motors); //Moves robot forward to push in glyph
        sleep(250);
        drive.encoderDrive(100, driveStyle.BACKWARD, 0.5, motors); //Moves robot backward
        //
    }

    @Override
    public void runOpMode(){ //Dummy method for class to work

    }
}

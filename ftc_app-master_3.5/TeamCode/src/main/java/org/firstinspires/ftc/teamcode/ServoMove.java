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
        double servo0PositionDown = 0.61;
        double servo0PositionUp = 0.27;
        for(int x = 0; x < 38; x++)
        {
            servos[0].setPosition(servo0PositionDown);
            servo0PositionDown -= .01;
            sleep(10);
        }
        //arm down
        sleep(1500);
        if(color == 0 && alliance.equals("blue"))
        {
            servos[1].setPosition(0); //Knocks off jewel
            sleep(750);
            servos[1].setPosition(0.20); //Moves slapper back
        }
        if(color == 0 && alliance.equals("red"))
        {
            servos[1].setPosition(0.50); //Knocks off jewel
            sleep(750);
            servos[1].setPosition(0.40); //Moves slapper back
        }
        if(color == 1 && alliance.equals("blue"))
        {
            servos[1].setPosition(0.70); //Knocks off jewel
            sleep(750);
            servos[1].setPosition(0.40); //Moves slapper back
        }
        if(color == 1 && alliance.equals("red"))
        {
            servos[1].setPosition(0.); //Knocks off jewl
            sleep(750);
            servos[1].setPosition(0.20); //Moves slapper back
        }

        if(color == 2)
        {
            servos[1].setPosition(0.20); //Slapper stays mid
        }

        sleep(750);
        for(int y = 0; y < 38; y++)
        {
            servos[0].setPosition(servo0PositionUp);
            servo0PositionUp += .01;
            sleep(10);
        } //Raises flapper up
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
        servos[3].setPosition(0.0); //Opens claw
        sleep(1250);
        drive.encoderDrive(200, driveStyle.BACKWARD, Drive.drivePower, motors); //Moves robot back
        sleep(750);
        servos[2].setPosition(0.15); //Moves knock to a mid position
        sleep(250);
        servos[3].setPosition(0.25); //Closes claw
        servos[2].setPosition(0.75); //Moves knock to home position
        sleep(500);
        drive.encoderDrive(850, driveStyle.FORWARD, Drive.drivePower, motors); //Moves robot forward to push in glyph
        sleep(250);
        drive.encoderDrive(250, driveStyle.BACKWARD, Drive.drivePower, motors); //Moves robot backward
        //
    }

    @Override
    public void runOpMode(){ //Dummy method for class to work

    }
}

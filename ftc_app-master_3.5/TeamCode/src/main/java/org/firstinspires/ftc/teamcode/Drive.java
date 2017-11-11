package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;


/**
 * Created by User on 10/7/2017.
 */


enum driveStyle
{
    FORWARD, BACKWARD, STRAFE_LEFT, STRAFE_RIGHT, FORWARD_RIGHT, FORWARD_LEFT, BACKWARD_RIGHT, BACKWARD_LEFT, PIVOT_RIGHT, PIVOT_LEFT
}
public class Drive extends LinearOpMode
{


    public double[] setPower(double dirX, double dirY, double pivot)
    {
        double[] motorPowers = new double[4];
        motorPowers[0] = (-dirY - dirX) - pivot;
        motorPowers[1] = -(-dirX + dirY) - pivot;
        motorPowers[2] = -(-dirY - dirX) - pivot;
        motorPowers[3] = (-dirX + dirY) - pivot;
        //

        //motorPowers[0] = motorRF
        //motorPowers[1] = motorRB
        //motorPowers[2] = motorLB
        //motorPowers[3] = motorLF

        return motorPowers;
    }


    public boolean encoderDrive(int encoderDelta, driveStyle drive, double motorPower, double timeout, DcMotor[] motors)
    {
        //ElapsedTime runtime = new ElapsedTime();


        switch(drive)
        {
            case FORWARD:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                motors[0].setPower(setPower(0, -motorPower, 0)[0]);
                motors[1].setPower(setPower(0, -motorPower, 0)[1]);
                motors[2].setPower(setPower(0, -motorPower, 0)[2]);
                motors[3].setPower(setPower(0, -motorPower, 0)[3]);

                while (motors[2].getCurrentPosition() >= (-encoderDelta + encoderReadingLB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;


            }

            case BACKWARD:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                motors[0].setPower(setPower(0, motorPower, 0)[0]);
                motors[1].setPower(setPower(0, motorPower, 0)[1]);
                motors[2].setPower(setPower(0, motorPower, 0)[2]);
                motors[3].setPower(setPower(0, motorPower, 0)[3]);

                while (motors[2].getCurrentPosition() <= (encoderDelta + encoderReadingLB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }

            case STRAFE_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                while (motors[2].getCurrentPosition() >= (encoderReadingLB - encoderDelta))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }

            case STRAFE_RIGHT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                motors[0].setPower(setPower(motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(motorPower, 0, 0)[3]);

                while (motors[2].getCurrentPosition() >= (-encoderDelta + encoderReadingLB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }

            case FORWARD_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                motors[0].setPower(setPower(-motorPower, -motorPower, 0)[0]);
                motors[1].setPower(setPower(-motorPower, -motorPower, 0)[1]);
                motors[2].setPower(setPower(-motorPower, -motorPower, 0)[2]);
                motors[3].setPower(setPower(-motorPower, -motorPower, 0)[3]);

                while (motors[2].getCurrentPosition() <= (encoderDelta + encoderReadingLB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }

            case FORWARD_RIGHT:
            {
                double encoderReadingRB = motors[1].getCurrentPosition();
                motors[0].setPower(setPower(motorPower, -motorPower, 0)[0]);
                motors[1].setPower(setPower(motorPower, -motorPower, 0)[1]);
                motors[2].setPower(setPower(motorPower, -motorPower, 0)[2]);
                motors[3].setPower(setPower(motorPower, -motorPower, 0)[3]);

                while (motors[1].getCurrentPosition() >= (-encoderDelta + encoderReadingRB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }

            case BACKWARD_LEFT:
            {
                double encoderReadingRB = motors[1].getCurrentPosition();
                motors[0].setPower(setPower(-motorPower, motorPower, 0)[0]);
                motors[1].setPower(setPower(-motorPower, motorPower, 0)[1]);
                motors[2].setPower(setPower(-motorPower, motorPower, 0)[2]);
                motors[3].setPower(setPower(-motorPower, motorPower, 0)[3]);

                while (motors[1].getCurrentPosition() <= (encoderDelta + encoderReadingRB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }

            case BACKWARD_RIGHT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                motors[0].setPower(setPower(motorPower, motorPower, 0)[0]);
                motors[1].setPower(setPower(motorPower, motorPower, 0)[1]);
                motors[2].setPower(setPower(motorPower, motorPower, 0)[2]);
                motors[3].setPower(setPower(motorPower, motorPower, 0)[3]);

                while (motors[2].getCurrentPosition() >= (-encoderDelta + encoderReadingLB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }

            case PIVOT_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                motors[0].setPower(setPower(0, 0, -motorPower)[0]);
                motors[1].setPower(setPower(0, 0, -motorPower)[1]);
                motors[2].setPower(setPower(0, 0, -motorPower)[2]);
                motors[3].setPower(setPower(0, 0, -motorPower)[3]);

                while (motors[2].getCurrentPosition() >= (-encoderDelta + encoderReadingLB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }

            case PIVOT_RIGHT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                motors[0].setPower(setPower(0, 0, motorPower)[0]);
                motors[1].setPower(setPower(0, 0, motorPower)[1]);
                motors[2].setPower(setPower(0, 0, motorPower)[2]);
                motors[3].setPower(setPower(0, 0, motorPower)[3]);

                while (motors[2].getCurrentPosition() <= (encoderDelta + encoderReadingLB))
                {

                }


                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                break;
            }


        }

       return true;
    }

    public void timeDrive(long time, double motorPower, driveStyle drive, DcMotor[] motors)
    {
        switch(drive)
        {
            case FORWARD:
                {
                    motors[0].setPower(setPower(0, -motorPower, 0)[0]);
                    motors[1].setPower(setPower(0, -motorPower, 0)[1]);
                    motors[2].setPower(setPower(0, -motorPower, 0)[2]);
                    motors[3].setPower(setPower(0, -motorPower, 0)[3]);

                    sleep(time);

                    motors[0].setPower(setPower(0, 0, 0)[0]);
                    motors[1].setPower(setPower(0, 0, 0)[1]);
                    motors[2].setPower(setPower(0, 0, 0)[2]);
                    motors[3].setPower(setPower(0, 0, 0)[3]);

                    break;


                }

            case BACKWARD:
            {
                motors[0].setPower(setPower(0, motorPower, 0)[0]);
                motors[1].setPower(setPower(0, motorPower, 0)[1]);
                motors[2].setPower(setPower(0, motorPower, 0)[2]);
                motors[3].setPower(setPower(0, motorPower, 0)[3]);

                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);

                break;

            }

            case STRAFE_LEFT:
            {
                motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);

                break;
            }

            case STRAFE_RIGHT:
            {
                motors[0].setPower(setPower(motorPower, 0, 0)[0]);
                motors[1].setPower(setPower(motorPower, 0, 0)[1]);
                motors[2].setPower(setPower(motorPower, 0, 0)[2]);
                motors[3].setPower(setPower(motorPower, 0, 0)[3]);

                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                break;
            }

            case FORWARD_LEFT:
            {
                motors[0].setPower(setPower(-motorPower, -motorPower, 0)[0]);
                motors[1].setPower(setPower(-motorPower, -motorPower, 0)[1]);
                motors[2].setPower(setPower(-motorPower, -motorPower, 0)[2]);
                motors[3].setPower(setPower(-motorPower, -motorPower, 0)[3]);

                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                break;
            }

            case FORWARD_RIGHT:
            {
                motors[0].setPower(setPower(motorPower, -motorPower, 0)[0]);
                motors[1].setPower(setPower(motorPower, -motorPower, 0)[1]);
                motors[2].setPower(setPower(motorPower, -motorPower, 0)[2]);
                motors[3].setPower(setPower(motorPower, -motorPower, 0)[3]);

                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                break;
            }

            case BACKWARD_LEFT:
            {
                motors[0].setPower(setPower(-motorPower, motorPower, 0)[0]);
                motors[1].setPower(setPower(-motorPower, motorPower, 0)[1]);
                motors[2].setPower(setPower(-motorPower, motorPower, 0)[2]);
                motors[3].setPower(setPower(-motorPower, motorPower, 0)[3]);

                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                break;
            }

            case BACKWARD_RIGHT:
            {
                motors[0].setPower(setPower(motorPower, motorPower, 0)[0]);
                motors[1].setPower(setPower(motorPower, motorPower, 0)[1]);
                motors[2].setPower(setPower(motorPower, motorPower, 0)[2]);
                motors[3].setPower(setPower(motorPower, motorPower, 0)[3]);

                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                break;
            }

            case PIVOT_LEFT:
            {
                motors[0].setPower(setPower(0, 0, -motorPower)[0]);
                motors[1].setPower(setPower(0, 0, -motorPower)[1]);
                motors[2].setPower(setPower(0, 0, -motorPower)[2]);
                motors[3].setPower(setPower(0, 0, -motorPower)[3]);



                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                break;
            }

            case PIVOT_RIGHT:
            {
                motors[0].setPower(setPower(0, 0, motorPower)[0]);
                motors[1].setPower(setPower(0, 0, motorPower)[1]);
                motors[2].setPower(setPower(0, 0, motorPower)[2]);
                motors[3].setPower(setPower(0, 0, motorPower)[3]);



                sleep(time);

                motors[0].setPower(setPower(0, 0, 0)[0]);
                motors[1].setPower(setPower(0, 0, 0)[1]);
                motors[2].setPower(setPower(0, 0, 0)[2]);
                motors[3].setPower(setPower(0, 0, 0)[3]);
                break;
            }
        }

    }

    @Override
    public void runOpMode()
    {}


}

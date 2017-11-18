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


    public boolean encoderDrive(int encoderDelta, driveStyle drive, double motorPower, DcMotor[] motors)
    {
        //ElapsedTime runtime = new ElapsedTime();


        switch(drive)
        {
            case FORWARD:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB - encoderDelta);

                forward(motorPower, motors);

                while (motors[2].getCurrentPosition() >= target)
                {

                }




                break;


            }

            case BACKWARD:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderDelta + encoderReadingLB);
                backward(motorPower, motors);

                while (motors[2].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case STRAFE_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB - encoderDelta);
                strafeLeft(motorPower, motors);

                while (motors[2].getCurrentPosition() >= target)
                {

                }


                break;
            }

            case STRAFE_RIGHT:
            {
                double encoderReadingLB = motors[1].getCurrentPosition();
                double target = (encoderReadingLB + encoderDelta);
                strafeRight(motorPower, motors);

                while (motors[1].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case FORWARD_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB - encoderDelta);
                forwardLeft(motorPower, motors);
                while (motors[2].getCurrentPosition() >= target)
                {

                }


                break;
            }

            case FORWARD_RIGHT:
            {
                double encoderReadingRB = motors[1].getCurrentPosition();
                double target = (encoderReadingRB + encoderDelta);
                forwardRight(motorPower, motors);

                while (motors[1].getCurrentPosition() <= target)
                {

                }

//
                break;
            }

            case BACKWARD_LEFT:
            {
                double encoderReadingRB = motors[1].getCurrentPosition();
                double target = (encoderDelta - encoderReadingRB);
                backwardLeft(motorPower, motors);

                while (motors[1].getCurrentPosition() >= target)
                {

                }


                break;
            }

            case BACKWARD_RIGHT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB + encoderDelta);
                backwardRight(motorPower, motors);

                while (motors[2].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case PIVOT_LEFT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderReadingLB + encoderDelta);
                pivotLeft(motorPower, motors);

                while (motors[2].getCurrentPosition() <= target)
                {

                }


                break;
            }

            case PIVOT_RIGHT:
            {
                double encoderReadingLB = motors[2].getCurrentPosition();
                double target = (encoderDelta - encoderReadingLB);
                pivotRight(motorPower, motors);

                while (motors[2].getCurrentPosition() >= target)
                {

                }


                break;
            }


        }

        motors[0].setPower(setPower(0, 0, 0)[0]);
        motors[1].setPower(setPower(0, 0, 0)[1]);
        motors[2].setPower(setPower(0, 0, 0)[2]);
        motors[3].setPower(setPower(0, 0, 0)[3]);

       return true;
    }

    public void timeDrive(long time, double motorPower, driveStyle drive, DcMotor[] motors)
    {
        switch(drive)
        {
            case FORWARD:
                {
                    forward(motorPower, motors);

                    sleep(time);


                    break;


                }

            case BACKWARD:
            {
                backward(motorPower, motors);

                sleep(time);


                break;

            }

            case STRAFE_LEFT:
            {
                strafeLeft(motorPower, motors);

                sleep(time);


                break;
            }

            case STRAFE_RIGHT:
            {
                strafeRight(motorPower, motors);

                sleep(time);

                break;
            }

            case FORWARD_LEFT:
            {
                forwardLeft(motorPower, motors);

                sleep(time);

                break;
            }

            case FORWARD_RIGHT:
            {
                forwardRight(motorPower, motors);


                sleep(time);

                break;
            }

            case BACKWARD_LEFT:
            {
                backwardLeft(motorPower, motors);

                sleep(time);

                break;
            }

            case BACKWARD_RIGHT:
            {
                backwardRight(motorPower, motors);

                sleep(time);

                break;
            }

            case PIVOT_LEFT:
            {
                pivotLeft(motorPower, motors);




                sleep(time);

                break;
            }

            case PIVOT_RIGHT:
            {
                pivotRight(motorPower, motors);


                sleep(time);


                break;
            }
        }


        motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
        motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
        motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
        motors[3].setPower(setPower(-motorPower, 0, 0)[3]);

    }

    public void forward(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(0, -motorPower, 0)[0]);
        motors[1].setPower(setPower(0, -motorPower, 0)[1]);
        motors[2].setPower(setPower(0, -motorPower, 0)[2]);
        motors[3].setPower(setPower(0, -motorPower, 0)[3]);
    }
    public void backward(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(0, motorPower, 0)[0]);
        motors[1].setPower(setPower(0, motorPower, 0)[1]);
        motors[2].setPower(setPower(0, motorPower, 0)[2]);
        motors[3].setPower(setPower(0, motorPower, 0)[3]);
    }
    public void strafeLeft(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(-motorPower, 0, 0)[0]);
        motors[1].setPower(setPower(-motorPower, 0, 0)[1]);
        motors[2].setPower(setPower(-motorPower, 0, 0)[2]);
        motors[3].setPower(setPower(-motorPower, 0, 0)[3]);
    }
    public void strafeRight(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(motorPower, 0, 0)[0]);
        motors[1].setPower(setPower(motorPower, 0, 0)[1]);
        motors[2].setPower(setPower(motorPower, 0, 0)[2]);
        motors[3].setPower(setPower(motorPower, 0, 0)[3]);
    }
    public void forwardLeft(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(-motorPower, -motorPower, 0)[0]);
        motors[1].setPower(setPower(-motorPower, -motorPower, 0)[1]);
        motors[2].setPower(setPower(-motorPower, -motorPower, 0)[2]);
        motors[3].setPower(setPower(-motorPower, -motorPower, 0)[3]);
    }
    public void forwardRight(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(motorPower, -motorPower, 0)[0]);
        motors[1].setPower(setPower(motorPower, -motorPower, 0)[1]);
        motors[2].setPower(setPower(motorPower, -motorPower, 0)[2]);
        motors[3].setPower(setPower(motorPower, -motorPower, 0)[3]);
    }
    public void backwardLeft(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(-motorPower, motorPower, 0)[0]);
        motors[1].setPower(setPower(-motorPower, motorPower, 0)[1]);
        motors[2].setPower(setPower(-motorPower, motorPower, 0)[2]);
        motors[3].setPower(setPower(-motorPower, motorPower, 0)[3]);
    }
    public void backwardRight(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(motorPower, motorPower, 0)[0]);
        motors[1].setPower(setPower(motorPower, motorPower, 0)[1]);
        motors[2].setPower(setPower(motorPower, motorPower, 0)[2]);
        motors[3].setPower(setPower(motorPower, motorPower, 0)[3]);
    }
    public void pivotLeft(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(0, 0, -motorPower)[0]);
        motors[1].setPower(setPower(0, 0, -motorPower)[1]);
        motors[2].setPower(setPower(0, 0, -motorPower)[2]);
        motors[3].setPower(setPower(0, 0, -motorPower)[3]);

    }
    public void pivotRight(double motorPower, DcMotor[] motors)
    {
        motors[0].setPower(setPower(0, 0, motorPower)[0]);
        motors[1].setPower(setPower(0, 0, motorPower)[1]);
        motors[2].setPower(setPower(0, 0, motorPower)[2]);
        motors[3].setPower(setPower(0, 0, motorPower)[3]);
    }
    @Override
    public void runOpMode()
    {}


}

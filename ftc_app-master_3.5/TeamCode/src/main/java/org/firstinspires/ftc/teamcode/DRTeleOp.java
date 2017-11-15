package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by User on 10/17/2017.
 */

@TeleOp(name = "DRTeleOp", group = "Testing")
public class DRTeleOp extends LinearOpMode
{
    RobotHardware curiosity = new RobotHardware();


    double slapperInit = 0.8;
    double slapperPosition = slapperInit;
    double flapperInit = 1.0;
    double flapperPosition = flapperInit;
    double wristInit = 0.375;
    double knockInit = 0.702;
    double clawInit = 0.93;
    double wristMaxChange = 0.005;
    double knockMaxChange = 0.005;
    double clawMaxChange = 0.005;
    double wristPos = wristInit;
    double knockPos = knockInit;
    double clawPos = clawInit;
    double zSclae = 0.75;

    double armServoAdjustment = 0.2;
    double joint1MaxSpeed = 0.70;
    double joint2MaxSpeed = 0.50;
    double joint3MaxSpeed = 1.0;

    double j3Low = 0;
    double j3High = 0;
    double joint3Power = 0.0;
    double j3Hold = 0.0;
    double j3AdjHold = 0.0;
    boolean j3HoldON = false;

    int count = 0;

    int j3EncoderChange = 0;
    int j3EncoderTarget = 0;
    int j3currentEncoder = 0;
    boolean j3MoveModeLast = true;
    boolean j3MoveMode = true;
    double speed = 1.0;

    boolean dPadUpState = false;
    boolean dPadDownState = false;


    public void runOpMode()
    {
        curiosity.init(hardwareMap);

        //Sets the end behavior of the motors - brake implies the robot will physically attempt to stop instead of just coasting
        curiosity.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        curiosity.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        curiosity.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        curiosity.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        curiosity.joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        curiosity.joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        curiosity.joint3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        curiosity.slapper.setPosition(slapperInit);
        curiosity.flapper.setPosition(flapperInit);
        curiosity.wrist.setPosition(wristInit);
        curiosity.knock.setPosition(knockInit);
        curiosity.claw.setPosition(clawInit);

        curiosity.joint3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curiosity.joint3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        j3currentEncoder = curiosity.joint3.getCurrentPosition();

        waitForStart();

        while (opModeIsActive())
        {
            //Clipping the ranges of servos so they don't go out of bounds
            slapperPosition = Range.clip(slapperPosition, 0.05, 0.95);
            flapperPosition = Range.clip(flapperPosition, 0.05, 0.95);
            armServoAdjustment = Range.clip(armServoAdjustment, 0.2, 0.7);
            knockPos = Range.clip(knockPos, 0.01, 0.702);
            wristPos = Range.clip(wristPos, 0.01, 0.99);


            if(gamepad1.a)
            {
                speed = 1.0;
            }
            else if(gamepad1.b)
            {
                speed = 0.5;
            }

            //Setting drive motors for mecanum - gamepad 1 - driver
            curiosity.motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zSclae * gamepad1.right_stick_x)));
            curiosity.motorLB.setPower(speed*(-(-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zSclae * gamepad1.right_stick_x)));
            curiosity.motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zSclae * gamepad1.right_stick_x)));
            curiosity.motorLF.setPower(speed*(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zSclae * gamepad1.right_stick_x));

            //Driver - Flapper and Slapper manual controls
            if(gamepad1.right_trigger > 0.5)
            {
                slapperPosition =+ 0.05;
            }
            else if(gamepad1.right_bumper)
            {
                slapperPosition  =- 0.05;
            }

            if(gamepad1.y)
            {
                flapperPosition =+ 0.05;
            }
            else if(gamepad1.x)
            {
                flapperPosition =- 0.05;
            }

            //Manipulator - Arm controls, Joints 1 and 2

            //Setting Arm Adjustment Speed
            if(gamepad2.dpad_up && !dPadUpState)
            {
                dPadUpState = true;
                armServoAdjustment += 0.02;
            }
            else if(!gamepad2.dpad_up)
            {
                dPadUpState = false;
            }

            if(gamepad2.dpad_down && !dPadDownState)
            {
                dPadDownState  = true;
                armServoAdjustment -= 0.02;
            }
            else if(!gamepad2.dpad_down)
            {
                dPadDownState = false;
            }

            //Setting Joint 3
            //Getting current encoder value for j3
            j3currentEncoder = curiosity.joint3.getCurrentPosition();
            //Setting Power Ranges
            if((j3currentEncoder >= -1000) && (j3currentEncoder <= 200))
            {
                j3High = .75;
                j3Low = .55;
                j3Hold = .02;
            }
            else if((j3currentEncoder >= 200) && (j3currentEncoder <= 600))
            {
                j3High = .40;
                j3Low = .20;
                j3Hold = .02;
            }
            else if((j3currentEncoder >= 600) && (j3currentEncoder <= 712))
            {
                j3High = -.75;
                j3Low = -.55;
                j3Hold = -.02;
            }
            else
            {
                j3High = 0.0;
                j3Low = 0.0;
                j3Hold = 0.0;
            }
            //Or j3Hold = fancy formula for value


            if(Math.abs(gamepad2.left_stick_y) < 0.1)
            {
                if (j3currentEncoder > 80)
                {

                    //Hold Position
                    j3MoveMode = false;
                    if (!j3MoveMode && j3MoveModeLast) //Transition from joystick movement to holding
                    {
                        count++;
                        j3EncoderTarget = j3currentEncoder;
                        j3AdjHold = j3Hold;
                    }

                    j3EncoderChange = j3currentEncoder - j3EncoderTarget; //Finding change in encoder values
                    if (j3EncoderChange >= 10)
                    {
                        if (j3currentEncoder < 400)
                        {
                            j3AdjHold -= .01;
                        } else
                        {
                            j3AdjHold += .01;
                        }
                    }
                    else if (j3EncoderChange <= -10)
                    {
                        if (j3currentEncoder > 400)
                        {
                            j3AdjHold += .01;
                        } else
                        {
                            j3AdjHold -= .01;
                        }
                    }
                    else
                    {
                        //j3AdjHold = j3Hold;
                    }
                    joint3Power = j3AdjHold;
                    //joint3Power = 0.0;
                }
                else
                {
                    j3Hold = 0.0;
                    telemetry.addData("Under 80 Encoder Counts", "Not Holding Power");
                    joint3Power = 0.0;
                }
            }
            else
            {
                //Not-Hold Position
                j3MoveMode = true;
                joint3Power = ((gamepad2.left_stick_y * 0.2) + j3Low);
            }
            curiosity.joint3.setPower(joint3Power);
            //Resetting Encoders for the next loop
            j3MoveModeLast = j3MoveMode;



            //Setting Joint 2
            if(gamepad2.left_bumper)
            {
                curiosity.joint2.setPower(joint2MaxSpeed);
            }
            else if(gamepad2.left_trigger > 0.4)
            {
                curiosity.joint2.setPower(-joint2MaxSpeed);
            }
            else
            {
                curiosity.joint2.setPower(0.0);
            }

            //Setting Joint 1
            if(gamepad2.dpad_up)
            {
                curiosity.joint1.setPower(joint3MaxSpeed);
            }
            else if(gamepad2.dpad_down)
            {
                curiosity.joint1.setPower(-joint3MaxSpeed);
            }
            else
            {
                curiosity.joint1.setPower(0.0);
            }

            //Removed capability of wrist for easier glyph control at first meet.
            /*
            if(gamepad2.right_stick_x > 0.2 || gamepad2.right_stick_x < -0.2)
            {
                wristPos += (gamepad2.right_stick_x * wristMaxChange);

                curiosity.wrist.setPosition(wristPos);

            }
            */

            if(gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2)
            {
                knockPos += gamepad2.right_stick_y * knockMaxChange;
                curiosity.knock.setPosition(knockPos);

            }

            if(gamepad2.right_bumper || gamepad2.right_trigger > 0.1)
            {
                if(gamepad2.right_bumper)
                {
                    clawPos += clawMaxChange;
                }
                if(gamepad2.right_trigger > 0.1)
                {
                    clawPos -= gamepad2.right_trigger * clawMaxChange;
                }

                curiosity.claw.setPosition(clawPos);

            }

            //Sending telemetry for arm data
            //telemetry.addData("armServoAdjustment", armServoAdjustment);
            //telemetry.addData("Joint 1", curiosity.joint1.getPower());
            //telemetry.addData("Joint 2", curiosity.joint2.getCurrentPosition());
            telemetry.addData("Joint 3 Low", j3Low);
            telemetry.addData("Joint 3 High", j3High);
            telemetry.addData("Joint 3 Hold", j3Hold);
            telemetry.addData("Joint 3 Encoder", curiosity.joint3.getCurrentPosition());
            telemetry.addData("Joint 3 Actual Power", curiosity.joint3.getPower());
            telemetry.addData("Joint 3 Target Encoder", j3EncoderTarget);
            telemetry.addData("Joint 3 Adjust Hold", j3AdjHold);
            telemetry.addData("Count", count);
            //telemetry.addData("Wrist Pos", curiosity.wrist.getPosition());
            //telemetry.addData("Knock Pos", curiosity.knock.getPosition());
            //telemetry.addData("Claw Pos", curiosity.claw.getPosition());

            telemetry.update();

            //Actually setting the positions of the servos
            curiosity.slapper.setPosition(slapperPosition);
            curiosity.flapper.setPosition(flapperPosition);
        }
    }
}

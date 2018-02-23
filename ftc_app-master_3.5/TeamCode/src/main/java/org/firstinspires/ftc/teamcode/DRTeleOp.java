package org.firstinspires.ftc.teamcode;

import android.app.ApplicationErrorReport;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by User on 10/17/2017.
 */

@TeleOp(name = "DRTeleOp", group = "Testing")
public class DRTeleOp extends LinearOpMode
{
    RobotHardware curiosity = new RobotHardware();


    /*double slapperInit = 0.8;
    double flapperInit = 1.0;
    double wristInit = 0.375;
    double knockInit = 0.702;
    double clawInit = 0.93;
    */
    double wristMaxChange = 0.005;
    double knockMaxChange = 0.005;
    double slapperMaxChange = 0.005;
    double flapperMaxChange = 0.004;
    double clawMaxChange = 0.005;
    double grabberMaxChange = 0.005;
    double grabberLiftMaxChange = 0.003;
    double zSclae = 0.75;

    double armServoAdjustment = 0.2;
    double joint1MaxSpeed = 0.70;
    double joint2MaxSpeed = 0.50;
    //double joint3MaxSpeed = 1.0;

    double slidesPower = 0.0;
    double grabberLiftPosition;
    double grabberPosition;

    boolean slidesEncoderCheck = true;

    double speed = 1.0;

    double clawOpen = 0.0;
    double knockSwitch;

    //double brakeOn = 0.10;
    //double brakeOff = 0.20;
    //double brakePosition;

    public void runOpMode() throws InterruptedException
    {
        // Sets initial servo positions, sets motor's mode to brake, and stops all the motors
        curiosity.init(hardwareMap);
        curiosity.slapper.setPosition(0.8);

        //curiosity.joint3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //curiosity.joint3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //j3currentEncoder = curiosity.joint3.getCurrentPosition();

        grabberLiftPosition = curiosity.grabberLift.getPosition();
        grabberPosition = curiosity.grabber.getPosition();
        double flapperPosition = curiosity.flapper.getPosition();
        double slapperPosition = curiosity.slapper.getPosition();
        //double wristPos = curiosity.wrist.getPosition();
        double knockPos = curiosity.knock.getPosition();
        double clawPos = curiosity.claw.getPosition();

        waitForStart();

        while (opModeIsActive())
        {
            //Clipping the ranges of servos so they don't go out of bounds
            slapperPosition = Range.clip(slapperPosition, 0.05, 0.95);
            flapperPosition = Range.clip(flapperPosition, 0.30, 0.70);
            armServoAdjustment = Range.clip(armServoAdjustment, 0.2, 0.7);
            knockPos = Range.clip(knockPos, 0.06, 0.75);
            //wristPos = Range.clip(wristPos, 0.01, 0.99);
            clawPos = Range.clip(clawPos, 0.01, 0.25);
            grabberLiftPosition = Range.clip(grabberLiftPosition, 0.18, 0.95);
            grabberPosition = Range.clip(grabberPosition, 0.01, 0.99);


            if(gamepad1.a)
            {
                speed = 1.0;
            }
            else if(gamepad1.b)
            {
                speed = 0.5;
            }

            //Setting drive motors for mecanum - gamepad 1 - driver
            curiosity.motorRF.setPower(speed*((-gamepad1.right_stick_y - gamepad1.right_stick_x) - (zSclae * gamepad1.left_stick_x)));
            curiosity.motorLB.setPower(speed*(-(-gamepad1.right_stick_y - gamepad1.right_stick_x) - (zSclae * gamepad1.left_stick_x)));
            curiosity.motorRB.setPower(speed*(-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (zSclae * gamepad1.left_stick_x)));
            curiosity.motorLF.setPower(speed*(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (zSclae * gamepad1.left_stick_x));

            /*curiosity.motorRF.setPower(speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zSclae * gamepad1.right_stick_x)));
            curiosity.motorLB.setPower(speed*(-(-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zSclae * gamepad1.right_stick_x)));
            curiosity.motorRB.setPower(speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zSclae * gamepad1.right_stick_x)));
            curiosity.motorLF.setPower(speed*(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zSclae * gamepad1.right_stick_x));
            */

            //Driver - Flapper and Slapper manual controls
            if(gamepad1.right_trigger > 0.5)
            {
                slapperPosition += slapperMaxChange;
            }
            else if(gamepad1.right_bumper)
            {
                slapperPosition  -= slapperMaxChange;
            }

            if(gamepad1.y)
            {
                flapperPosition += flapperMaxChange;
            }
            else if(gamepad1.x)
            {
                flapperPosition -= flapperMaxChange;
            }

            //Manipulator - Arm controls, Joints 1 and 2

            //Controlling GrabberLift Movement
            /*if(gamepad2.dpad_up && !dPadUpState)
            {
                dPadUpState = true;
                grabberLiftPosition += 0.05;
            }
            else if(!gamepad2.dpad_up)
            {
                dPadUpState = false;
            }

            if(gamepad2.dpad_down && !dPadDownState)
            {
                dPadDownState  = true;
                grabberLiftPosition -= 0.05;
            }
            else if(!gamepad2.dpad_down)
            {
                dPadDownState = false;
            }*/

            if(gamepad2.a)
            {
                grabberLiftPosition += grabberLiftMaxChange;
            }
            if(gamepad2.y)
            {
                grabberLiftPosition -= grabberLiftMaxChange;
            }
            if(gamepad2.b)
            {
                grabberLiftPosition = 0.535;
            }
            curiosity.grabberLift.setPosition(grabberLiftPosition);

            //Controlling Grabber Open/Close
            /*
            if(gamepad2.right_bumper || gamepad2.right_trigger > 0.1)
            {
                //Opening the Grabber
                if(gamepad2.right_bumper)
                {
                    grabberPosition -= (grabberMaxChange);
                }
                //Closing the Grabber
                if(gamepad2.right_trigger > 0.1)
                {
                    grabberPosition += (gamepad2.right_trigger * grabberMaxChange);
                }
            }
            */
            /*if(gamepad2.guide)
            {
                slidesEncoderCheck = false;
            }*/

            if(gamepad2.right_trigger > 0.9)
            {
                grabberPosition = 0.99;
                curiosity.grabber.setPosition(0.99);
            }
            if(gamepad2.x)
            {
                grabberPosition = 0.005;
            }
            curiosity.grabber.setPosition(grabberPosition);

            //Controlling the Slides (Relic Arm Movement)
            if(gamepad2.right_stick_y > 0.1)
            {
                slidesPower = -gamepad2.right_stick_y * .40;
            }
            else if(gamepad2.right_stick_y < -0.1)
            {
                slidesPower = -((gamepad2.right_stick_y * 0.5) - 0.5);
            }
            else
            {
                curiosity.slides.setPower(0.0);
            }
            if(slidesEncoderCheck)
            {
                if((curiosity.slides.getCurrentPosition() < 4260 && gamepad2.right_stick_y < -0.1 ) || (curiosity.slides.getCurrentPosition() > 0 && gamepad2.right_stick_y > 0.1))
                {
                    curiosity.slides.setPower(slidesPower);
                }
                else
                {
                    curiosity.slides.setPower(0.0);
                }
            }
            else
            {
                curiosity.slides.setPower(slidesPower);
            }

            //Complete Relic Release Button
            if(gamepad2.dpad_down)
            {
                while(curiosity.grabberLift.getPosition() > .54)
                {
                    curiosity.grabberLift.setPosition(curiosity.grabberLift.getPosition() - 0.025);
                    sleep(50);
                }
                curiosity.grabberLift.setPosition(0.54);
                sleep(500);
                curiosity.grabber.setPosition(0.99);
                grabberPosition = 0.99;
                sleep(500);
                while(curiosity.slides.getCurrentPosition() > 2500)
                {
                    curiosity.slides.setPower(-0.40);
                }
                curiosity.slides.setPower(0.0);
                sleep(100);
                curiosity.grabberLift.setPosition(0.95);
            }

            if(gamepad2.dpad_up)
            {
                grabberLiftPosition = 0.95;
                curiosity.grabberLift.setPosition(0.95);
            }


            //Removed capability of wrist for easier glyph control at first meet.
            /*
            if(gamepad2.right_stick_x > 0.2 || gamepad2.right_stick_x < -0.2)
            {
                wristPos += (gamepad2.right_stick_x * wristMaxChange);

                curiosity.wrist.setPosition(wristPos);

            }
            */

            if(gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2)
            {

                knockPos += (gamepad2.left_stick_y * knockMaxChange);
                curiosity.knock.setPosition(knockPos);
            }

            //Setting the Claw
            if(gamepad2.left_bumper || gamepad2.left_trigger > 0.1)
            {
                if(gamepad2.left_bumper)
                {
                    clawPos += (clawMaxChange);
                }
                if(gamepad2.left_trigger > 0.1)
                {
                    clawPos -= (gamepad2.left_trigger * clawMaxChange);
                }

                curiosity.claw.setPosition(clawPos);
            }
            if(gamepad1.left_bumper)
            {
                clawPos = clawOpen;
                curiosity.claw.setPosition(clawPos);
            }

            /*if(gamepad1.back)
            {
                slapperPosition = 0.8;
                curiosity.slapper.setPosition(slapperPosition);
                sleep(500);
                flapperPosition = 0.67;
                curiosity.flapper.setPosition(flapperPosition);
                knockPos = 0.395;
                curiosity.knock.setPosition(knockPos);
                sleep(500);
                clawPos = 0.94;
                curiosity.claw.setPosition(clawPos);
                sleep(2000);
                knockPos = 0.75;
                curiosity.knock.setPosition(knockPos);
            }
            */

            /*if(gamepad2.dpad_left)
            {
                brakePosition += 0.01;
            }
            if(gamepad2.dpad_right)
            {
                brakePosition -= 0.01;
            }
            */

            //Sending telemetry for arm data
            //telemetry.addData("armServoAdjustment", armServoAdjustment);
            //telemetry.addData("Joint 1", curiosity.joint1.getPower());
            //telemetry.addData("Joint 2", curiosity.joint2.getCurrentPosition());
            //telemetry.addData("Joint 3 Low", j3Low);
            //telemetry.addData("Joint 3 High", j3High);
            //telemetry.addData("Joint 3 Hold", j3Hold);
            //telemetry.addData("Joint 3 Encoder", curiosity.joint3.getCurrentPosition());
            //telemetry.addData("Joint 3 Actual Power", curiosity.joint3.getPower());
            //telemetry.addData("Joint 3 Target Encoder", j3EncoderTarget);
            //telemetry.addData("Joint 3 Adjust Hold", j3AdjHold);
            //telemetry.addData("j3EncoderChange", j3EncoderChange);
            //telemetry.addData("j3armMotion", armMotion);
            //telemetry.addData("Count", count);
            //telemetry.addData("Wrist Pos", curiosity.wrist.getPosition());
            telemetry.addData("Knock Pos", curiosity.knock.getPosition());
            telemetry.addData("Claw Pos", curiosity.claw.getPosition());
            telemetry.addData("Grabber", curiosity.grabber.getPosition());
            telemetry.addData("GrabberLift", curiosity.grabberLift.getPosition());
            telemetry.addData("Slides Power", slidesPower);
            telemetry.addData("Slides Actual Motor Power", curiosity.slides.getPower());
            telemetry.addData("Slides Encoder", curiosity.slides.getCurrentPosition());
            telemetry.addData("Encoder Check?", slidesEncoderCheck);

            //telemetry.addData("Flapper Pos", curiosity.flapper.getPosition());
            //telemetry.addData("Slapper Pos", curiosity.slapper.getPosition());
            //telemetry.addData("Robot Speed", speed);
            //telemetry.addData("Guide Button Status", gamepad1.guide);
            //telemetry.addData("Back Button Status", gamepad1.back);
            //telemetry.addData("Brake Position", curiosity.brake.getPosition());

            telemetry.update();

            //Actually setting the positions of the servos
            curiosity.slapper.setPosition(slapperPosition);
            curiosity.flapper.setPosition(flapperPosition);
            //curiosity.brake.setPosition(brakePosition);
        }
    }
}

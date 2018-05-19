package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 5/19/2018.
 */
@TeleOp(name="SingleMotorTesting",group = "")

public class SingleVariableMotor extends LinearOpMode{
   DcMotor motor;
    boolean motorOn = false;
    double motorpower = -.5;
    public void runOpMode(){
        motor = hardwareMap.dcMotor.get("motor");
        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.y){
                motorpower -= 0.1;
                sleep(250);
            }
            if(gamepad1.a){
                motorpower += 0.1;
                sleep(250);

            }
            if(gamepad1.dpad_up){
                motorpower -= 0.01;
                sleep(250);

            }
            if(gamepad1.dpad_down){
                motorpower += 0.01;
                sleep(250);

            }

            if(gamepad1.b)
            {
                motorOn = false;
            }
            if(gamepad1.x)
            {
                motorOn = true;
            }

            if(motorOn)
            {
                motor.setPower(motorpower);
            }
            else
            {
                motor.setPower(0);
            }

            telemetry.addData("motorspeed:",motorpower);
            telemetry.update();
        }
    }
}

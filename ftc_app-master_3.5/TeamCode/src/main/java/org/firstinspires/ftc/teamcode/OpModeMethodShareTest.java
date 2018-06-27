package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by User on 4/26/2018.
 */
@Autonomous (name = "OpModeMethodShareTEest", group = "")
public class OpModeMethodShareTest extends LinearOpMode
{
    DcMotor motorRF = null;
    DcMotor motorLF = null;
    DcMotor motorRB = null;
    DcMotor motorLB = null;
    Drive_Mecanum drive = new Drive_Mecanum();
    VoltageSensor robotVoltage;
    double voltageNow = 0.0;
    public void runOpMode()
    {

        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//] Sets motors so when they have 0 power, they brake instead of coast

        DcMotor[] motors = new DcMotor[4];//]
        motors[0] = motorRF;//]
        motors[1] = motorRB;//] Array to hold all of the drive motors
        motors[2] = motorLB;//]
        motors[3] = motorLF;//]

        Drive_Mecanum.isMethodShareEncoderTestFinished = false;
        Drive_Mecanum.encoder = 0;
        Drive_Mecanum.startingEncoder = motors[1].getCurrentPosition();

        int loop = 0;

        waitForStart();

        while(opModeIsActive())
        {
            robotVoltage = hardwareMap.voltageSensor.get("Expansion Hub 2");
            //accessing expansion hub voltage
            voltageNow = robotVoltage.getVoltage();
            if(!Drive_Mecanum.isMethodShareEncoderTestFinished)
            {
                drive.methodShareEncoderTest(0.5, 5000, motors);
            }
            //drive.methodShareModPower(gamepad1.left_stick_y, motors);

            telemetry.addData("Current Voltage", voltageNow);
            telemetry.addData("MotorRB Pos", motorRB.getCurrentPosition());
            telemetry.addData("MotorLB Pos", motorLB.getCurrentPosition());
            telemetry.update();
        }
    }

}

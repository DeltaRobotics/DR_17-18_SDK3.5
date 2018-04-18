package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by User on 4/17/2018.
 */

@Autonomous (name = "StallDetectionTest", group = "")
public class StallDectectionTest extends LinearOpMode
{

    public static RobotHardware robot = new RobotHardware();

    ElapsedTime runtime = new ElapsedTime();

    Drive drive = new Drive();


    VoltageSensor robotVoltage;
    double voltageNow = 0.0;
    double startVoltage = 0.0;
    double stallVoltage = 0.0;
    double targetRuntime = 0.0;
    int loopCount = 0;

    boolean stall = false;



    @Override
    public void runOpMode()
    {

        robot.init(hardwareMap);
        DcMotor[] motors = new DcMotor[4];//]
        motors[0] = robot.motorRF;//]
        motors[1] = robot.motorRB;//] Array to hold all of the drive motors
        motors[2] = robot.motorLB;//]
        motors[3] = robot.motorLF;//]

        waitForStart();
        while (opModeIsActive() && !stall)
        {
            motors[0].setPower(setPower(0, 1, 0)[0]);
            motors[1].setPower(setPower(0, 1, 0)[1]);
            motors[2].setPower(setPower(0, 1, 0)[2]);
            motors[3].setPower(setPower(0, 1, 0)[3]);

            robotVoltage = hardwareMap.voltageSensor.get("Expansion Hub 10");

            voltageNow = robotVoltage.getVoltage();

            if(loopCount == 3)
            {
                startVoltage = voltageNow;
                stallVoltage = startVoltage - 2.0;
            }

            if(voltageNow <= stallVoltage)
            {
                if(targetRuntime == 0)
                {
                    targetRuntime = runtime.milliseconds() + 1000;
                }
                if(runtime.milliseconds() >= targetRuntime)
                {
                    stall = true;
                }
            }
            else
            {
                targetRuntime = 0;
            }
        }
    }
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
}


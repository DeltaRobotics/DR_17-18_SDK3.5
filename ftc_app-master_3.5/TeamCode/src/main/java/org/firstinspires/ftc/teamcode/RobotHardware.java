package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by User on 9/26/2017.
 */

public class RobotHardware
{
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;

    public DcMotor joint1 = null;
    public DcMotor joint2 = null;
    public DcMotor joint3 = null;

    public Servo wrist = null;
    public Servo knock = null;
    public Servo claw = null;

    public Servo flapper = null;
    public Servo slapper = null;

    boolean useEncoder = false;

    HardwareMap hwMap = null;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public RobotHardware()
    {

    }

    public void init(HardwareMap ahwMap)
    {
        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");

        joint1 = ahwMap.dcMotor.get("joint1");
        joint2 = ahwMap.dcMotor.get("joint2");
        joint3 = ahwMap.dcMotor.get("joint3");

        wrist = ahwMap.servo.get("wrist");
        knock = ahwMap.servo.get("knock");
        claw = ahwMap.servo.get("claw");

        flapper = ahwMap.servo.get("flapper");
        slapper = ahwMap.servo.get("slapper");


        motorRF.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);

        //

    }
}

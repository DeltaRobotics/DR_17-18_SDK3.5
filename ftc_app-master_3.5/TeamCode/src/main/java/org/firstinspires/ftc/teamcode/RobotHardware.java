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
    public DcMotor motorRF = null; //MotorRF on drive train
    public DcMotor motorLF = null; //MotorLF on drive train
    public DcMotor motorRB = null; //MotorRB on drive train
    public DcMotor motorLB = null; //MotorLB on drive train

    public DcMotor joint1 = null; //Joint 1 on arm
    public DcMotor joint2 = null; //Joint 2 on arm
    public DcMotor joint3 = null; //Joint 3 on arm

    public Servo wrist = null; //Wrist on arm
    public Servo knock = null; //Knock on arm
    public Servo claw = null; //Claw on arm
    public Servo brake = null; //Servo brake on arm

    public Servo flapper = null; //Flapper
    public Servo slapper = null; //Slapper

    public RobotHardware() //Constructor
    {

    }

    public void init(HardwareMap ahwMap) //Code that runs when you init the hardware map
    {
        motorRF = ahwMap.dcMotor.get("motorRF"); //What to look for in config for motorRF
        motorLF = ahwMap.dcMotor.get("motorLF"); //What to look for in config for motorLF
        motorRB = ahwMap.dcMotor.get("motorRB"); //What to look for in config for motorRB
        motorLB = ahwMap.dcMotor.get("motorLB"); //What to look for in the config for motorLB

        joint1 = ahwMap.dcMotor.get("joint1"); //What to look for in the config for joint1
        joint2 = ahwMap.dcMotor.get("joint2"); //What to look for in the config for joint2
        joint3 = ahwMap.dcMotor.get("joint3"); //What to look for in the config for joint3

        wrist = ahwMap.servo.get("wrist"); //What to look for in the config for wrist
        knock = ahwMap.servo.get("knock"); //What to look for in the config for knock
        claw = ahwMap.servo.get("claw"); //What to look for in the config for claw
        brake = ahwMap.servo.get("brake"); //What to look for in the config for brake

        flapper = ahwMap.servo.get("flapper"); //What to look for in the config for flapper
        slapper = ahwMap.servo.get("slapper"); //What to look for in the config for slapper

         // was 0.8 for sideways
        flapper.setPosition(0.67);//]
        wrist.setPosition(0.375);//]
        knock.setPosition(0.75);//] Sets servos to their home positions
        claw.setPosition(0.25);//]
        brake.setPosition(0.10);//]

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//] Sets motors so when they have 0 power, they brake instead of coast
        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]
        joint3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//]

        motorRF.setPower(0);//]
        motorLF.setPower(0);//] Stops the drive motors
        motorRB.setPower(0);//]
        motorLB.setPower(0);//]

    }
}

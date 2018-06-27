package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by User on 3/22/2018.
 */
//DO NOT MODIFY THIS! USE ONLY FOR TANK DRIVE TRAIN!
@TeleOp (name = "TankDrive", group = "")
public class TankDrive extends OpMode
{
    double shooterpower = 0;
    double sweeperpower = 0;
    int load_position = 400;
    int fire_position = 2048;
    DcMotor motorLF;
    DcMotor motorRF;
    DcMotor motorLB;
    DcMotor motorRB;
    DcMotor shooter;
    DcMotor sweeper;

    boolean loading = false;
    boolean firing = false;
    public void init()
    {
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter = hardwareMap.dcMotor.get("shooter");

        motorRB.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorLF.setPower(0);
        sweeper.setPower(0);
        shooter.setPower(0);

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop()
    {
        motorRB.setPower(gamepad1.left_stick_y);
        motorRF.setPower(gamepad1.left_stick_y);
        motorLF.setPower(-gamepad1.right_stick_y);
        motorLB.setPower(-gamepad1.right_stick_y);
        shooter.setPower(shooterpower);
        sweeper.setPower(sweeperpower);

        /*//encoder shooter
        if (gamepad1.a) {
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shooter.setPower(0.75);
            shooter.setTargetPosition(load_position);
            load_position += 2048;
            sleep(250);
        }
        if (gamepad1.b){
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shooter.setPower(0.75);
            shooter.setTargetPosition(fire_position);
            fire_position += 2048;
            sleep(250);
        }
        if (gamepad1.x) {
            shooterpower = 0;
        }
        */
        if(gamepad1.a)
        {
            loading = true;
        }

        if(gamepad1.b)
        {
            firing = true;
        }

        if((shooter.getCurrentPosition() < load_position) && loading)
        {
            shooterpower = 0.35;
        }
        else if(loading)
        {
            shooterpower = 0.15;
            loading = false;
            load_position += 1680;
        }

        if((shooter.getCurrentPosition() < fire_position) && firing)
        {
            shooterpower = 0.35;
        }
        else if(firing)
        {
            shooterpower = 0.0;
            firing = false;
            fire_position += 1680;
        }

        if (gamepad1.x) {
            shooterpower = 0;
        }

        //manuel shooter
        /*
        if (gamepad1.a) {
        motorpower = .40;
        }
        if (gamepad1.b) {
        motorpower = 0;
        }
        if (gamepad1.x){
            motorpower += .05;
        }
        if (gamepad1.y){
            motorpower -= .05 ;
        }
        */



        //sweeper
        if (gamepad1.dpad_up){
            sweeperpower = 1.;
        }
        if (gamepad1.dpad_down){
            sweeperpower = -1.;
        }
        if (gamepad1.dpad_left) {
            sweeperpower = 0;
        }

        shooter.setPower(shooterpower);
        sweeper.setPower(sweeperpower);

        telemetry.addData("load position", load_position);
        telemetry.addData("fire position", fire_position);
        telemetry.addData("sweeper", sweeper.getCurrentPosition());
        telemetry.addData("shooter", shooter.getCurrentPosition());
        telemetry.update();
    }
    private static void sleep(int time) // In milliseconds
    {
        double constant = System.currentTimeMillis();
        double current = System.currentTimeMillis();
        while ((current - constant) <= time) {
            current = System.currentTimeMillis();
        }
    }
}


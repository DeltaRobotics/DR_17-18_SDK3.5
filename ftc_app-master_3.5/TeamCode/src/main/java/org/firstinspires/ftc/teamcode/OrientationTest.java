package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by User on 12/9/2017.
 */
@Autonomous (name="OrientationTest", group = "")
public class OrientationTest extends LinearOpMode
{
    BNO055IMU imu;

    Orientation angles;


    Drive drive = new Drive();
    RobotHardware robot = new RobotHardware();
    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.temperatureUnit     = BNO055IMU.TempUnit.CELSIUS;
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Init Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        telemetry.update();
        waitForStart();
        /*drive.OrientationDrive(85, driveStyle.PIVOT_LEFT, 0.3, motors,imu);
        sleep(500);
        drive.OrientationDrive(95, driveStyle.PIVOT_RIGHT, 0.3, motors,imu);
        sleep(500);
        drive.OrientationDrive(50, driveStyle.PIVOT_RIGHT, 0.3, motors,imu);
        sleep(500);
        drive.OrientationDrive(45, driveStyle.PIVOT_LEFT, 0.3, motors,imu);
        */

        while(opModeIsActive()) {
            //sleep(3000);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            //sleep(2000);
            /*drive.OrientationDrive(6, driveStyle.PIVOT_LEFT, 0.4, motors, imu);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Orientation After Pivot", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            sleep(2000);
            */

        }
    }
}

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by User on 10/14/2017.
 */

@Autonomous (name = "AutoBlueLeft", group = "Auto")
public class AutoBlueLeft extends LinearOpModeCamera
{
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive();
    ServoMove servoMove = new ServoMove();
    BNO055IMU imu;
    Orientation angles;

    VuforiaLocalizer vuforia;
    String keyPosition;
    boolean vuforiaOn = true;
    boolean relicAnalysis = true;

    String color = "undecided";

    int jewelColorInt;

    double timeout = 0;

    @Override
    public void runOpMode()
    {

        robot.init(hardwareMap);
        robot.slapper.setPosition(0.3);

        robot.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;

        Servo[] servos = new Servo[4];
        servos[0] = robot.flapper;
        servos[1] = robot.slapper;
        servos[2] = robot.knock;
        servos[3] = robot.claw;

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parametersIMU.temperatureUnit     = BNO055IMU.TempUnit.CELSIUS;
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Init Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        telemetry.update();


        if (isCameraAvailable())
        {
            //Resolution of image, currently set to 1 (higher number means less resolution but faster speed)
            setCameraDownsampling(1);
            //Takes some time, is initializing all of the camera's internal workings
            startCamera();
            //Stays Initialized, waits for the Driver's Station Button to be pressed
            waitForStart();

            if (imageReady())
            {

                int redValueLeft = -76800;
                int blueValueLeft = -76800;
                int greenValueLeft = -76800;

                Bitmap rgbImage;
                //The last value must correspond to the downsampling value from above
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                telemetry.addData("Width", rgbImage.getWidth());
                telemetry.addData("Height", rgbImage.getHeight());
                telemetry.update();

                //This is for only saving the color image if needed.


                for (int x = 600; x < 900; x++) //Sets x bounds for visual box on picture saved to the phone
                {
                    for (int y = 950; y < 1280; y++) //Sets y bounds for visual box on picture saved to the phone
                    {
                        if (x == 879 && y >= 950) //Bounds for the line below
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0)); //Draws line on picture saved to the phone
                        }
                        if (x >= 0 && y == 950) //Bounds for the line below
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0)); //Draws line on picture saved to the phone
                        }
                        if (x == 600 && y >= 950) //Bounds for the line below
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0)); //Draws line on picture saved to the phone
                        }
                        if (x >= 600 && y == 1279) //Bounds for the line below
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0)); //Draws line on picture saved to the phone
                        }
                    }
                }

                SaveImage(rgbImage);

                //Analyzing Jewel Color
                for (int x = 600; x < 900; x++) //Sets x bounds for the box we will be analyzing
                {
                    for (int y = 950; y < 1280; y++) //Sets y bounds for the box we will be analyzing
                    {
                        int pixel = rgbImage.getPixel(x, y);
                        redValueLeft += red(pixel);
                        blueValueLeft += blue(pixel);
                        greenValueLeft += green(pixel);
                    }
                }
                redValueLeft = normalizePixels(redValueLeft);
                blueValueLeft = normalizePixels(blueValueLeft);
                greenValueLeft = normalizePixels(greenValueLeft);
                telemetry.addData("redValueLeft", redValueLeft);
                telemetry.addData("blueValueLeft", blueValueLeft);
                telemetry.addData("greenValueLeft", greenValueLeft);


                jewelColorInt = highestColor(redValueLeft, blueValueLeft, greenValueLeft);

                telemetry.addData("Jewel Color", jewelColorInt);
                if (jewelColorInt == 0)
                {
                    telemetry.addData("Jewel Color", "0 : Red");
                }
                else if (jewelColorInt == 1)
                {
                    telemetry.addData("Jewel Color", "1 : Blue");
                }
                else if (jewelColorInt == 2)
                {
                    telemetry.addData("Jewel Color", "Green? What Did You Do? Green Shouldn't Even Be An Option!");
                } else
                {
                    telemetry.addData("Jewel Color", "Something's Wrong");
                }
                telemetry.update();
            }
            stopCamera();

            if(vuforiaOn)
            {
                //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                parameters.vuforiaLicenseKey = "ARab//j/////AAAAGa3dGFLc9ECfpTtxg0azy4sjU1xxnDSHmo2gKPM2BecEH5y5QNOI7fiEsflqB1" +
                        "9dYDi655Mj6avzS4Vru7PegjjQCH1YVLwUZ4iX80Q02P0S+cA9Vw71hoZoI8nMdLgvgplYFv/M3ofqFezhHE7Afc9fq/ixLzl4P5d" +
                        "z61T+SR43HzNb7At7XC3z9cSLqHD2ba+WWbKUPf6bcivgqimS8ekVeZHubkwfIqFVxXGZEfSScTfGa0/3l5/TaBpaUoUkz+JhAULt" +
                        "pt2PwYdpCfhdCP3eo+2a8DJjP3eSXlCkuEoAUtUCUzCXWxS+pHDHCyUtEAxf8LaKvSh3aYoO7dNzmh4TspC3mFVrLbyZMzii8GgC";

                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

                VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                VuforiaTrackable relicTemplate = relicTrackables.get(0);

                relicTrackables.activate();

                timeout = System.currentTimeMillis() + 4000;
                keyPosition = "UNKNOWN";
                while (opModeIsActive() && relicAnalysis && System.currentTimeMillis() < timeout)
                {
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                    {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        relicAnalysis = false;
                        if(vuMark == RelicRecoveryVuMark.CENTER)
                        {
                            keyPosition = "CENTER";
                        }
                        if(vuMark == RelicRecoveryVuMark.LEFT)
                        {
                            keyPosition = "LEFT";
                        }
                        if(vuMark == RelicRecoveryVuMark.RIGHT)
                        {
                            keyPosition = "RIGHT";
                        }
                    }
                    else
                    {
                        telemetry.addData("VuMark", "UNKNOWN visible");
                        keyPosition = "UNKNOWN";
                    }
                    telemetry.update();
                }
            }
            //drive.timeDrive(750, 0.4, driveStyle.STRAFE_LEFT, motors);
            drive.encoderDrive(450, driveStyle.STRAFE_LEFT, 0.45, motors);
            sleep(250);
            //drive.timeDrive(800, 0.5, driveStyle.STRAFE_RIGHT, motors);
            drive.encoderDrive(200, driveStyle.STRAFE_RIGHT, 0.45, motors);
        /*sleep(2000);
        robot.slapper.setPosition(0.3);
        sleep(1000);
        robot.flapper.setPosition(0.68);
        sleep(1000);
        color = "blue";
        if(color == "blue")
        {
            robot.slapper.setPosition(0.10);
        }
        if(color == "red")
        {
            robot.slapper.setPosition(0.50);
        }
        sleep(1000);
        robot.slapper.setPosition(0.3);
        sleep(1000);
        robot.flapper.setPosition(1.0);
        sleep(1000);
        robot.slapper.setPosition(0.8);
        sleep(1000);
        */
            servoMove.knockOffJewel(servos, jewelColorInt, "blue");
            sleep(250);
            //drive.timeDrive(85, 0.4, driveStyle.STRAFE_LEFT, motors);
            //
            drive.encoderDrive(50, driveStyle.STRAFE_LEFT, 0.45, motors);
            sleep(250);
            //drive.timeDrive(800, 0.5, driveStyle.FORWARD, motors);
            drive.encoderDrive(1250, driveStyle.FORWARD, 0.5, motors);
            sleep(250);
            //drive.timeDrive(750, 0.5, driveStyle.STRAFE_RIGHT, motors);

            switch(keyPosition)
            {
                case "LEFT":
                {
                    drive.encoderDrive(600, driveStyle.STRAFE_RIGHT, 0.45, motors);
                    break;
                }

                case "CENTER":
                {
                    drive.encoderDrive(1050, driveStyle.STRAFE_RIGHT, 0.55, motors);
                    break;
                }

                case "RIGHT":
                {
                    drive.encoderDrive(1550
                            , driveStyle.STRAFE_RIGHT, 0.55, motors);
                    break;
                }

                case "UNKNOWN":
                {
                    drive.encoderDrive(1000, driveStyle.STRAFE_RIGHT, 0.55, motors);
                    break;
                }
            }
            sleep(500);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Before Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            if(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > 0)
            {
                telemetry.update();
                drive.OrientationDrive((Math.abs(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)))/2, driveStyle.PIVOT_RIGHT, 0.4, motors, imu);
            }
            else
            {
                telemetry.update();
                drive.OrientationDrive((Math.abs(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)))/2, driveStyle.PIVOT_LEFT, 0.4, motors, imu);
            }
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            servoMove.placeGlyph(servos, robot, drive);


        }
    }
}



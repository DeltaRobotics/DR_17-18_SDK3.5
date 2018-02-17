package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous (name = "AutoRedRightDiagonal", group = "Auto Diagonal")
public class AutoRedRightDiagonal extends LinearOpModeCamera
{
    RobotHardware robot = new RobotHardware(); //Object of RobotHardware class
    Drive drive = new Drive(); //Object of Drive class
    ServoMove servoMove = new ServoMove(); //Object of ServoMove class
    BNO055IMU imu; //Rev IMU sensor
    Orientation angles; //Object for the robots orientation

    VuforiaLocalizer vuforia; //Vuforia object
    String keyPosition; //String for where the key is in the cryptobox
    boolean vuforiaOn = true; //Sets wether vuforia is on ??
    boolean relicAnalysis = true; //Sets wether we should analyzie the relic ??

    String color = "undecided"; //String that tells us which color the jewel under the pictogram is

    int jewelColorInt; //Integer variable for the jewel color so the code can use it
    boolean firstTime = true;
    boolean cameraAgain = true;
    int jewelLeftX = 380;
    int[]jewelCentering;
    boolean pictureFound = false;
    int jewelAdjust = 1400;

    double pivotValue = 0;


    double timeout = 0;

    double targetError = 0;


    @Override
    public void runOpMode()
    {

        robot.init(hardwareMap); //Inits hardware map
        robot.slapper.setPosition(0.3); //Sets slapper to the special home position for auto

        DcMotor[] motors = new DcMotor[4];//]
        motors[0] = robot.motorRF;//]
        motors[1] = robot.motorRB;//] Array to hold all of the drive motors
        motors[2] = robot.motorLB;//]
        motors[3] = robot.motorLF;//]

        Servo[] servos = new Servo[4];//]
        servos[0] = robot.flapper;//]
        servos[1] = robot.slapper;//] Array to hold some servos we use in auto
        servos[2] = robot.knock;//]
        servos[3] = robot.claw;//]

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters(); //Declares parameters object forIMU
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES; //Sets the unit in which we measure orientation in degrees
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //Sets acceleration unit in meters per second ??
        parametersIMU.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode, sets what file the IMU ueses
        parametersIMU.loggingEnabled      = true; //Sets wether logging in enable
        parametersIMU.loggingTag          = "IMU"; //Sets logging tag
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //Sets acceleration integration algorithm
        parametersIMU.temperatureUnit     = BNO055IMU.TempUnit.CELSIUS; //Sets units for temperature readings
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //Inits IMU
        imu.initialize(parametersIMU); //Init IMU parameters (set above)
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
        telemetry.addData("Init Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays initial orientation
        telemetry.update(); //Updates telemetry


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
                for (int x = 550; x < 900; x++) //Sets x bounds for the box we will be analyzing
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
                        if (vuMark == RelicRecoveryVuMark.CENTER)
                        {
                            keyPosition = "CENTER";
                        }
                        if (vuMark == RelicRecoveryVuMark.LEFT)
                        {
                            keyPosition = "LEFT";
                        }
                        if (vuMark == RelicRecoveryVuMark.RIGHT)
                        {
                            keyPosition = "RIGHT";
                        }
                    } else
                    {
                        telemetry.addData("VuMark", "UNKNOWN visible");
                        keyPosition = "UNKNOWN";
                    }
                    telemetry.update();
                }
                /*
                CameraDevice.getInstance().stop();
                CameraDevice.getInstance().deinit();
                */
            }

            //drive.encoderDrive(450, driveStyle.STRAFE_LEFT, 0.45, motors); //Strafes off stone
            //sleep(250);
            //drive.encoderDrive(200, driveStyle.STRAFE_RIGHT, 0.45, motors); //Strafes back to hit stone
            sleep(250);
            /*
            if (firstTime)
            {
                startCamera();
                CameraDevice.getInstance().init();
                firstTime = false;
            }
            while (opModeIsActive() && cameraAgain)
            {
                if (imageReady() && !pictureFound)
                {
                    Bitmap rgbImage2;
                    //The last value must correspond to the downsampling value from above
                    rgbImage2 = convertYuvImageToRgb(yuvImage, width, height, 1);
                    jewelCentering = FindJewelsCenter(rgbImage2);
                    for (int x = 0; x < 960; x++)
                    {
                        for (int y = 0; y < 1280; y++)
                        {
                            if (y % 100 == 0)
                            {
                                rgbImage2.setPixel(x, y, Color.rgb(120, 255, 255));
                            }
                            if (x % 100 == 0)
                            {
                                rgbImage2.setPixel(x, y, Color.rgb(120, 255, 255));
                            }
                            if(x == 319)
                            {
                                rgbImage2.setPixel(x, y, Color.rgb(255, 0, 0));
                            }
                            if(x == 413)
                            {
                                rgbImage2.setPixel(x, y, Color.rgb(255, 0, 0));
                            }
                            if (y == 848)
                            {
                                rgbImage2.setPixel(x, y, Color.rgb(255, 0, 0));
                            }
                            if (y == 852)
                            {
                                rgbImage2.setPixel(x, y, Color.rgb(255, 0, 0));
                            }
                            if(jewelCentering[x] == 0)
                            {
                                rgbImage2.setPixel(x, 850, Color.rgb(255,255,255));
                                rgbImage2.setPixel(x, 851, Color.rgb(255,255,255));
                                rgbImage2.setPixel(x, 849, Color.rgb(255,255,255));
                            }
                            if(jewelCentering[x] == 1)
                            {
                                rgbImage2.setPixel(x, 850, Color.rgb(10,10,10));
                                rgbImage2.setPixel(x, 851, Color.rgb(10,10,10));
                                rgbImage2.setPixel(x, 849, Color.rgb(10,10,10));
                            }
                            if(jewelCentering[x] == 2)
                            {
                                jewelLeftX = x;
                                pictureFound = true;
                                rgbImage2.setPixel(x, 850, Color.rgb(255,127,40));
                                rgbImage2.setPixel(x, 851, Color.rgb(255,127,40));
                                rgbImage2.setPixel(x, 849, Color.rgb(255,127,40));
                            }
                        }
                    }
                    telemetry.update();
                    SaveImage(rgbImage2);
                    telemetry.addData("Jewel Left X", jewelLeftX);
                    if(jewelLeftX > 413)
                    {
                        jewelAdjust = jewelLeftX - 413;

                    }
                    if(jewelLeftX < 319)
                    {
                        jewelAdjust = jewelLeftX - 319;
                    }
                    telemetry.addData("Adjust", jewelAdjust);

                    telemetry.update();
                }
                stopCamera();
                cameraAgain = false;
            }
            if(jewelAdjust < 0 && jewelAdjust > -100)
            {
                drive.encoderDrive(75, driveStyle.BACKWARD, 0.30, motors);
            }
            else if(jewelAdjust < 0)
            {
                drive.encoderDrive(150, driveStyle.BACKWARD, 0.30, motors);
            }
            if(jewelAdjust > 0 && jewelAdjust < 100)
            {
                drive.encoderDrive(75, driveStyle.FORWARD, 0.30, motors);
            }
            else if(jewelAdjust > 0 && jewelAdjust != 1400)
            {
                drive.encoderDrive(150, driveStyle.FORWARD, 0.30, motors);
            }
            */
            servoMove.knockOffJewel(servos, jewelColorInt, "red"); //Knocks off correct jewel
            //sleep(250);
            //drive.encoderDrive(50, driveStyle.STRAFE_LEFT, 0.45, motors); //Strafes so the robot isn't right against the stone
            sleep(250);
            drive.encoderDrive(1150, driveStyle.BACKWARD, Drive.drivePower, motors); //Drives forward towards the cryptobox
            sleep(250);

            switch(keyPosition) //Handles where the robot should move depending on key
            {
                //Subtracted 350 from each to compensate
                case "LEFT": //If key is LEFT
                {
                    //Was 600
                    //drive.encoderDrive(100, driveStyle.STRAFE_RIGHT, Drive.strafePower, motors); //Strafes to left column of cryptobox
                    pivotValue = 150;
                    break;
                }

                case "CENTER": //If key is CENTER
                {
                    //Was 1050
                    //drive.encoderDrive(550, driveStyle.STRAFE_RIGHT, Drive.strafePower, motors); //Strafes to center column of cryptobox
                    pivotValue = 130;
                    break;
                }

                case "RIGHT": //If key is RIGHT
                {
                    //Was 1550
                    //drive.encoderDrive(1050, driveStyle.STRAFE_RIGHT, Drive.strafePower, motors); //Strafes to right column of cryptobox
                    pivotValue = 170;
                    break;
                }

                case "UNKNOWN": //If phone couldn't sense a pictogram
                {
                    //Was 1050
                    //drive.encoderDrive(550, driveStyle.STRAFE_RIGHT, Drive.strafePower, motors); //Strafes to center column
                    pivotValue = 120;
                    break;
                }
            }
            drive.OrientationDrive(pivotValue, driveStyle.PIVOT_RIGHT, Drive.pivotPower, motors, imu);
            sleep(500);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
            telemetry.addData("Before Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays current orientation before the robot pivots
            telemetry.update(); //Updates telemetry
            //Chunk of code below corrects the robot's orientation
            if(!(keyPosition == "LEFT")) {
                targetError = (pivotValue - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) / 2;
                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > (pivotValue + 3) || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < (pivotValue - 3)) {
                    if (targetError < 0) //If the robot's current orientation is greater than 0
                    {
                        //telemetry.addData("Expected Change 1", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) / 2);
                        telemetry.update(); //Updates telemetry
                        drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, Drive.pivotPower, motors, imu); //Moves robot to correct orientation
                    } else //If the robot's current orientation isn't greater than 0
                    {
                        //telemetry.addData("Expected Change 2", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) / 2);
                        telemetry.update(); //Updates telemetry
                        drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_RIGHT, Drive.pivotPower, motors, imu); //Moves robot to correct orientation
                    }
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
                    telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after the orientation correction
                    telemetry.update(); //Updates telemetry
                }
            }

            servoMove.placeGlyph(servos, robot, drive); //Places and pushes in the glyph into the correct cryptobox column
            sleep(250);
            drive.encoderDrive(300, driveStyle.FORWARD, Drive.drivePower, motors); //Moves robot forward to push in glyph
            sleep(250);
            drive.encoderDrive(50, driveStyle.BACKWARD, Drive.drivePower, motors); //Moves robot backward


        }
    }
}



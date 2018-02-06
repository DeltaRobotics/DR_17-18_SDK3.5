package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

//@Autonomous (name = "AutoBlueRight", group = "Auto")
public class AutoBlueRight extends LinearOpModeCamera {
    RobotHardware robot = new RobotHardware(); //Object of RobotHardware class
    Drive drive = new Drive(); //Object of Drive class
    ServoMove servoMove = new ServoMove(); //Object of ServoMove class

    BNO055IMU imu; //BNO055IMU sensor in Rev Module

    Orientation angles; //Special object

    String color = "undecided"; //String that holds the jewel under the pictograms color
    VuforiaLocalizer vuforia; //Vuforia object
    String keyPosition; //String that holds the key position of the cryptobox
    boolean vuforiaOn = true; //Sets whether vuforia is on or not ??
    boolean relicAnalysis = true; //Sets whether the robot should analyze the relic or not

    int jewelColorInt; //Integer equivalent of the jewel under the pictogram's color
    double timeout = 0;

    double targetError = 0; //How many degrees the robot is off in orientation after we pivot

    @Override
    public void runOpMode() //Runs OpMode
    {

        robot.init(hardwareMap); //Inits hardware map
        robot.slapper.setPosition(0.3); //Moves slapper to its special home position for auto

        DcMotor[] motors = new DcMotor[4];//]
        motors[0] = robot.motorRF;//]
        motors[1] = robot.motorRB;//] Array that hold the drive train motors
        motors[2] = robot.motorLB;//]
        motors[3] = robot.motorLF;//]

        Servo[] servos = new Servo[4];//]
        servos[0] = robot.flapper;//]
        servos[1] = robot.slapper;//] Array that holds some servos used in auto
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


        if (isCameraAvailable()) {
            //Resolution of image, currently set to 1 (higher number means less resolution but faster speed)
            setCameraDownsampling(1);
            //Takes some time, is initializing all of the camera's internal workings
            startCamera();
            //Stays Initialized, waits for the Driver's Station Button to be pressed
            waitForStart();

            if (imageReady()) {

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

                for (int x = 480; x < 680; x++)
                {
                    for (int y = 850; y < 1280; y++)
                    {
                        if (x == 679 && y >= 850)
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x >= 0 && y == 850)
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x == 480 && y >= 850)
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x >= 480 && y == 1279)
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                    }
                }

                SaveImage(rgbImage);

                //Analyzing Jewel Color
                for (int x = 480; x < 680; x++) {
                    for (int y = 850; y < 1280; y++) {
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
                if (jewelColorInt == 0) {
                    telemetry.addData("Jewel Color", "0 : Red");
                } else if (jewelColorInt == 1) {
                    telemetry.addData("Jewel Color", "1 : Blue");
                } else if (jewelColorInt == 2) {
                    telemetry.addData("Jewel Color", "Green? What Did You Do? Green Shouldn't Even Be An Option!");
                } else {
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

            sleep(250);
            drive.encoderDrive(450, driveStyle.STRAFE_LEFT, 0.45, motors); //Robot strafes off stone
            sleep(250);
            drive.encoderDrive(200, driveStyle.STRAFE_RIGHT, 0.45, motors); //Robot strafes back into stone
            servoMove.knockOffJewel(servos, jewelColorInt, "blue"); //Robot knocks off correct jewel
            sleep(250);
            drive.encoderDrive(50, driveStyle.STRAFE_LEFT, 0.45, motors); //Robot strafes a little so it is not against the stone
            sleep(250);
            switch (keyPosition) //Handles where the robot should move depending on the key
            {
                case "RIGHT": //If key is RIGHT
                {
                    drive.encoderDrive(2625, driveStyle.FORWARD, 0.5, motors); //Moves robot to the right column of the cryptobox
                    break;
                }

                case "CENTER": //If key is CENTER
                {
                    drive.encoderDrive(2100, driveStyle.FORWARD, 0.5, motors); //Moves robot to the center column of the cryptobox
                    break;
                }

                case "LEFT": //If key is LEFT
                {
                    drive.encoderDrive(1625, driveStyle.FORWARD, 0.5, motors); //Moves robot to the left column of the cryptobox
                    break;
                }

                case "UNKNOWN": //If the phone didn't sense a pictograph
                {
                    drive.encoderDrive(2200, driveStyle.FORWARD, 0.5, motors); //Moves robot to the center column of the cryptobox
                    break;
                }
            }

            sleep(250);
            drive.encoderDrive(200, driveStyle.STRAFE_RIGHT, 0.45, motors); //Moves robot away from the cryptobox so it can rotate without hitting it
            sleep(250);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets the robot's current orientation
            drive.OrientationDrive(85 + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle), driveStyle.PIVOT_LEFT, 0.4, motors, imu); //Pivots robot 90 degrees right

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets the robot's current orientation
            telemetry.addData("Before Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays the robots orientation before it trys to correct its orientation
            telemetry.update(); //Updates telemetry
            if(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < 87 || AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > 93)
            {
                targetError = (90 - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) / 2; //Sets target error so the robot corrects its orientation properly
                if (targetError > 0) //If target error is greater than 0
                {
                    telemetry.update(); //Updates telemetry
                    drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_LEFT, 0.4, motors, imu); //Corrects robot's orientation
                } else //If target error isn't greater than 0
                {
                    telemetry.update(); //Updates telemetry
                    drive.OrientationDrive(Math.abs(targetError), driveStyle.PIVOT_RIGHT, 0.4, motors, imu); //Corrects robot's orientation
                }
            }
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets robot's orientation
            telemetry.addData("After Move", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays robot's orientation after it tries to correct its orientation
            sleep(250);
            drive.encoderDrive(300, driveStyle.BACKWARD, 0.5, motors); //Moves the robot backwards a bit
            sleep(250);
            servoMove.placeGlyph(servos, robot, drive); //Places glyph in correct cryptobox column
        }
    }
}



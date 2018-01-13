package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Camera;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;
import com.vuforia.CameraDevice;
import com.vuforia.DataSet;
import com.vuforia.ObjectTracker;
import com.vuforia.Tracker;
import com.vuforia.TrackerManager;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by User on 10/28/2017.
 */
@Autonomous(name="Jewel Only", group ="Autonomous Testing")
public class JustCameraJewel extends LinearOpModeCamera
{
    VuforiaLocalizer vuforia;
    int jewelColorInt;
    String keyPosition;

    boolean vuforiaOn = true;
    boolean relicAnalysis = true;
    boolean firstTime = true;
    boolean cameraAgain = true;
    int jewelRightX = 1400;
    int[]jewelColor;
    long timeStart;
    long timeElapsed;

    public void runOpMode()
    {
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

                Bitmap rgbImage1;
                //The last value must correspond to the downsampling value from above
                rgbImage1 = convertYuvImageToRgb(yuvImage, width, height, 1);

                //Looks at the size of the rgbImage for troubleshooting
                //telemetry.addData("Width", rgbImage.getWidth());
                //telemetry.addData("Height", rgbImage.getHeight());
                telemetry.update();

                //This is for only saving the color image if needed.

                //Sets the vertical boundaries of the camera image
                for (int x = 480; x < 680; x++)
                {
                    //Sets the horizontal boundaries of the camera image
                    for (int y = 850; y < 1280; y++)
                    {
                        //Sets the right boundary of the image to a certain color
                        if (x == 679 && y >= 850)
                        {
                            rgbImage1.setPixel(x, y, Color.rgb(0, 255, 255));
                        }
                        //Sets the upper boundary of the image to a certain color
                        if (x >= 0 && y == 850)
                        {
                            rgbImage1.setPixel(x, y, Color.rgb(0, 255, 255));
                        }
                        //Sets the left boundary of the image to a certain color
                        if (x == 480 && y >= 850)
                        {
                            rgbImage1.setPixel(x, y, Color.rgb(0, 255, 255));
                        }
                        //Sets the lower boundary of the image to a certain color
                        if (x >= 0 && y == 1279)
                        {
                            rgbImage1.setPixel(x, y, Color.rgb(0, 255, 255));
                        }
                    }
                }

                //SaveImage(rgbImage1);

                //Analyzing Jewel Color

                //Sets the vertical boundaries of the camera image analysis
                for (int x = 480; x < 680; x++)
                {
                    //Sets the horizontal boundaries of the camera image analysis
                    for (int y = 850; y < 1280; y++)
                    {
                        //Adds the value of each pixel to the aggregate color value (red/blue/green)
                        int pixel = rgbImage1.getPixel(x, y);
                        redValueLeft += red(pixel);
                        blueValueLeft += blue(pixel);
                        greenValueLeft += green(pixel);
                    }
                }
                //Divides each aggregate color by a large number to make the values easier to understand and compare
                redValueLeft = normalizePixels(redValueLeft);
                blueValueLeft = normalizePixels(blueValueLeft);
                greenValueLeft = normalizePixels(greenValueLeft);
                //telemetry.addData("redValueLeft", redValueLeft);
                //telemetry.addData("blueValueLeft", blueValueLeft);
                //telemetry.addData("greenValueLeft", greenValueLeft);


                //Determines the highest color to determine jewel color
                jewelColorInt = highestColor(redValueLeft, blueValueLeft, greenValueLeft);

                //Prints the jewelColorInt variable to a string based on the color of the jewel
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
                }
                else
                {
                    telemetry.addData("Jewel Color", "Something's Wrong");
                }
                telemetry.update();
            }
            stopCamera();

            if (vuforiaOn)
            {
                //Initializing the Vuforia camera parameters
                //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                //The license key of Vuforia (specific to 9925)
                parameters.vuforiaLicenseKey = "ARab//j/////AAAAGa3dGFLc9ECfpTtxg0azy4sjU1xxnDSHmo2gKPM2BecEH5y5QNOI7fiEsflqB1" +
                        "9dYDi655Mj6avzS4Vru7PegjjQCH1YVLwUZ4iX80Q02P0S+cA9Vw71hoZoI8nMdLgvgplYFv/M3ofqFezhHE7Afc9fq/ixLzl4P5d" +
                        "z61T+SR43HzNb7At7XC3z9cSLqHD2ba+WWbKUPf6bcivgqimS8ekVeZHubkwfIqFVxXGZEfSScTfGa0/3l5/TaBpaUoUkz+JhAULt" +
                        "pt2PwYdpCfhdCP3eo+2a8DJjP3eSXlCkuEoAUtUCUzCXWxS+pHDHCyUtEAxf8LaKvSh3aYoO7dNzmh4TspC3mFVrLbyZMzii8GgC";

                //Initialization of Vuforia camera and localizer (more background init)
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

                //Retrieves the VuMark files that we are looking for
                VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                VuforiaTrackable relicTemplate = relicTrackables.get(0);


                //Activates the VuMarks we want (The pictographs)
                relicTrackables.activate();

                //This loop only runs while a relic VuMark has not been found and the OpMode is running
                while (opModeIsActive() && relicAnalysis)
                {
                    //Sets the keyPosition variable to whichever picture is seen by Vuforia
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
                    }
                    else
                    {
                        telemetry.addData("VuMark", "UNKNOWN visible");
                        keyPosition = "UNKNOWN";
                    }
                    telemetry.update();
                }


                //Our own variable for Vuforia - whether it's used or not
                vuforiaOn = false;
                //Deactivates the camera that is currently being used
                CameraDevice.getInstance().stop();
                CameraDevice.getInstance().deinit();
                telemetry.addData("Camera Device", CameraDevice.getInstance());
                //Deinitializes Vuforia itself (not needed? May avoid having to deal with unclosed data sets?)
                // Vuforia.deinit();

            }
            if(firstTime)
            {
                startCamera();
                CameraDevice.getInstance().init();

                firstTime = false;
            }

            while(opModeIsActive() && cameraAgain)
            {

                if (imageReady())
                {

                    timeStart = System.currentTimeMillis();
                    while (timeElapsed < 15000)
                    {
                        Bitmap rgbImage2;
                        //The last value must correspond to the downsampling value from above
                        rgbImage2 = convertYuvImageToRgb(yuvImage, width, height, 1);
                        jewelColor = FindJewelsCenter(rgbImage2);

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
                                if (y == 848)
                                {
                                    rgbImage2.setPixel(x, y, Color.rgb(255, 0, 0));
                                }
                                if (y == 852)
                                {
                                    rgbImage2.setPixel(x, y, Color.rgb(255, 0, 0));
                                }
                                if(jewelColor[x] == 0)
                                {
                                    rgbImage2.setPixel(x, 850, Color.rgb(255,255,255));
                                    rgbImage2.setPixel(x, 851, Color.rgb(255,255,255));
                                    rgbImage2.setPixel(x, 849, Color.rgb(255,255,255));
                                }
                                if(jewelColor[x] == 1)
                                {
                                    rgbImage2.setPixel(x, 850, Color.rgb(10,10,10));
                                    rgbImage2.setPixel(x, 851, Color.rgb(10,10,10));
                                    rgbImage2.setPixel(x, 849, Color.rgb(10,10,10));
                                }
                                if(jewelColor[x] == 2)
                                {
                                    rgbImage2.setPixel(x, 850, Color.rgb(255,127,40));
                                    rgbImage2.setPixel(x, 851, Color.rgb(255,127,40));
                                    rgbImage2.setPixel(x, 849, Color.rgb(255,127,40));
                                }
                            }
                        }
                        telemetry.update();
                        SaveImage(rgbImage2);
                        timeElapsed = System.currentTimeMillis() - timeStart;
                        telemetry.addData("Jewel Right X", jewelRightX);
                        telemetry.update();
                    }
                    telemetry.addData("Out of", "loop");
                }
                stopCamera();
            }
            stopCamera();


        }


    }

}


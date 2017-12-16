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

                //telemetry.addData("Width", rgbImage.getWidth());
                //telemetry.addData("Height", rgbImage.getHeight());
                telemetry.update();

                //This is for only saving the color image if needed.

                for (int x = 480; x < 680; x++)
                {
                    for (int y = 850; y < 1280; y++)
                    {
                        if (x == 679 && y >= 850)
                        {
                            rgbImage1.setPixel(x, y, Color.rgb(0, 255, 255));
                        }
                        if (x >= 0 && y == 850)
                        {
                            rgbImage1.setPixel(x, y, Color.rgb(0, 255, 255));
                        }
                        if (x == 480 && y >= 850)
                        {
                            rgbImage1.setPixel(x, y, Color.rgb(0, 255, 255));
                        }
                        if (x >= 0 && y == 1279)
                        {
                            rgbImage1.setPixel(x, y, Color.rgb(0, 255, 255));
                        }
                    }
                }

                //SaveImage(rgbImage1);

                //Analyzing Jewel Color
                for (int x = 480; x < 680; x++)
                {
                    for (int y = 850; y < 1280; y++)
                    {
                        int pixel = rgbImage1.getPixel(x, y);
                        redValueLeft += red(pixel);
                        blueValueLeft += blue(pixel);
                        greenValueLeft += green(pixel);
                    }
                }
                redValueLeft = normalizePixels(redValueLeft);
                blueValueLeft = normalizePixels(blueValueLeft);
                greenValueLeft = normalizePixels(greenValueLeft);
                //telemetry.addData("redValueLeft", redValueLeft);
                //telemetry.addData("blueValueLeft", blueValueLeft);
                //telemetry.addData("greenValueLeft", greenValueLeft);


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

                while (opModeIsActive() && relicAnalysis)
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
                    }
                    else
                    {
                        telemetry.addData("VuMark", "UNKNOWN visible");
                        keyPosition = "UNKNOWN";
                    }
                    telemetry.update();
                }


                vuforiaOn = false;
                //relicTrackables.deactivate();
                CameraDevice.getInstance().stop();
                CameraDevice.getInstance().deinit();
                telemetry.addData("Camera Device", CameraDevice.getInstance());
                //Vuforia.deinit();

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

                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;

                    Bitmap rgbImage2;
                    //The last value must correspond to the downsampling value from above
                    rgbImage2 = convertYuvImageToRgb(yuvImage, width, height, 1);

                    for (int x = 0; x < 960; x++)
                    {
                        for (int y = 800; y < 1280; y++)
                        {
                            if (y == 1229)
                            {
                                rgbImage2.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                            if (y == 1231)
                            {
                                rgbImage2.setPixel(x, y, Color.rgb(0, 255, 0));
                            }
                        }
                    }
                    SaveImage(rgbImage2);
                    //telemetry.addData("Width", rgbImage.getWidth());
                    //telemetry.addData("Height", rgbImage.getHeight());
                    telemetry.update();

                    while (jewelRightX > 100)
                    {
                        jewelRightX = FindJewelsCenter(rgbImage2);
                        telemetry.addData("Jewel Right X", jewelRightX);
                        telemetry.update();
                    }
                }
                stopCamera();
            }
            stopCamera();


        }


    }

}


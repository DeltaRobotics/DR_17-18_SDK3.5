package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by User on 10/28/2017.
 */
@Autonomous(name="Cryptobox Testing", group ="Autonomous Testing")
public class CryptoboxVisionAnalysis extends LinearOpModeCamera
{
    boolean target = false;
    long timeStart;
    long timeElapsed;
    int count = 0;


    public void runOpMode()
    {
        //Resolution of image, currently set to 1 (higher number means less resolution but faster speed)
        setCameraDownsampling(1);
        //Takes some time, is initializing all of the camera's internal workings
        startCamera();
        //Stays Initialized, waits for the Driver's Station Button to be pressed
        waitForStart();

        while (opModeIsActive() && !target)
        {

            if (imageReady())
            {
                Bitmap rgbImage2;
                //The last value must correspond to the downsampling value from above
                rgbImage2 = convertYuvImageToRgb(yuvImage, width, height, 1);
                FindCryptoboxSides(rgbImage2, "RED");

                for (int x = 0; x < 960; x++)
                {
                    for (int y = 0; y < 1280; y++)
                    {
                        if (y == height - (int) (height * .25))
                        {
                            rgbImage2.setPixel(x, y, Color.rgb(204, 102, 0));
                        }
                        if (y == height - (int) (height * .5))
                        {
                            rgbImage2.setPixel(x, y, Color.rgb(204, 102, 0));
                        }
                        if (y == height - (int) (height * .75))
                        {
                            rgbImage2.setPixel(x, y, Color.rgb(204, 102, 0));
                        }

                    }
                }
                count++;
                telemetry.addData("Count", count);
                telemetry.update();
                SaveImage(rgbImage2);
                timeElapsed = System.currentTimeMillis() - timeStart;
            }
        }
        stopCamera();


    }
}


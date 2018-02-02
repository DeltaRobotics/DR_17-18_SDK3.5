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
    int cryptoboxHeight;
    int cryptoboxWidth;
    int[] resultingMovement = new int[5];


    public void runOpMode()
    {
        //Resolution of image, currently set to 1 (higher number means less resolution but faster speed)
        setCameraDownsampling(0);
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
                rgbImage2 = convertYuvImageToRgb(yuvImage, width, height, 0);
                resultingMovement = FindCryptoboxSides(rgbImage2, "RED");
                cryptoboxHeight = rgbImage2.getHeight();
                cryptoboxWidth = rgbImage2.getWidth();

                for (int x = 0; x < cryptoboxWidth; x++)
                {
                    for (int y = 0; y < cryptoboxHeight; y++)
                    {
                        if (y == cryptoboxHeight - (int) (cryptoboxHeight * .25))
                        {
                            rgbImage2.setPixel(x, y, Color.rgb(204, 102, 0));
                        }
                        if (y == cryptoboxHeight - (int) (cryptoboxHeight * .5))
                        {
                            rgbImage2.setPixel(x, y, Color.rgb(204, 102, 0));
                        }
                        if (y == cryptoboxHeight - (int) (cryptoboxHeight * .75))
                        {
                            rgbImage2.setPixel(x, y, Color.rgb(204, 102, 0));
                        }
                        if (y % 100 == 0)
                        {
                            rgbImage2.setPixel(x, y, Color.rgb(200, 255, 255));
                        }
                        if (x % 100 == 0)
                        {
                            rgbImage2.setPixel(x, y, Color.rgb(200, 255, 255));
                        }

                    }
                }
                count++;
                telemetry.addData("LinearValue 0", resultingMovement[0]);
                telemetry.addData("VisionAnalysisValue 0", cryptoboxHeight - (int) (cryptoboxHeight * .25));
                telemetry.addData("LinearValue 1", resultingMovement[1]);
                telemetry.addData("VisionAnalysisValue 1", cryptoboxHeight - (int) (cryptoboxHeight * .5));
                telemetry.addData("LinearValue 2", resultingMovement[2]);
                telemetry.addData("VisionAnalysisValue 2", cryptoboxHeight - (int) (cryptoboxHeight * .75));
                telemetry.addData("Count", count);
                telemetry.update();
                SaveImage(rgbImage2);
                timeElapsed = System.currentTimeMillis() - timeStart;
            }
        }
        stopCamera();


    }
}


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;
import android.view.View;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;

import for_camera_opmodes.LinearOpModeCamera;
import for_camera_opmodes.OpModeCamera;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by User on 9/30/2017.
 */

@Autonomous(name="Jewel and Target", group ="Autonomous Testing")
public class JewelAndVisionTargetTesting extends LinearOpModeCamera
{

    int jewelColorInt;

    //public View screenshot;


    //OpenGLMatrix lastLocation = null;
    //Initializing Vuforia Analysis
    //VuforiaLocalizer vuforia;

    public void runOpMode()
    {

        //Both camera methods utilize the monitor of the Robot Controller Phone. In order to see the color of the jewel, we are not sending the vuforia data below.
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //Used instead of the Robot Controller Monitor data
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //Our Vuforia Liscence Key - obtained through the Vuforia Developer Website
        parameters.vuforiaLicenseKey = "ARab//j/////AAAAGa3dGFLc9ECfpTtxg0azy4sjU1xxnDSHmo2gKPM2BecEH5y5QNOI7fiEsflqB1" +
                "9dYDi655Mj6avzS4Vru7PegjjQCH1YVLwUZ4iX80Q02P0S+cA9Vw71hoZoI8nMdLgvgplYFv/M3ofqFezhHE7Afc9fq/ixLzl4P5d" +
                "z61T+SR43HzNb7At7XC3z9cSLqHD2ba+WWbKUPf6bcivgqimS8ekVeZHubkwfIqFVxXGZEfSScTfGa0/3l5/TaBpaUoUkz+JhAULt" +
                "pt2PwYdpCfhdCP3eo+2a8DJjP3eSXlCkuEoAUtUCUzCXWxS+pHDHCyUtEAxf8LaKvSh3aYoO7dNzmh4TspC3mFVrLbyZMzii8GgC";

        //Determining which camera on the phone to use. FRONT refers to the front-facing image (less resolution but better range for the image)
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Tells the Vuforia interface exactly what we are looking for - loads in the Relics' VuMarks
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

*/
        //Resolution of image, currently set to 1 (higher number means less resolution but faster speed)
        setCameraDownsampling(1);
        //Takes some time, is initializing all of the camera's internal workings
        startCamera();
        //Stays Initialized, waits for the Driver's Station Button to be pressed
        waitForStart();

        //Activates Vuforia to begin searching for the Relic VuMark images
        //relicTrackables.activate();

        while (opModeIsActive())
        {
            telemetry.addData("Active", "OpMode");
            if (isCameraAvailable())
            {
                /*telemetry.addData("Relic", "Analysis");
                //RELIC ANALYSIS
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                {

                //Found an instance of the template. In the actual game, you will probably
                 //loop until this condition occurs, then move on to act accordingly depending
                 // on which VuMark was visible.
                    telemetry.addData("VuMark", "%s visible", vuMark);

                 //For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 //it is perhaps unlikely that you will actually need to act on this pose information, but
                 //we illustrate it nevertheless, for completeness.
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                //We further illustrate how to decompose the pose into useful rotational and
                 //translational components
                    if (pose != null)
                    {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }
                } else
                {
                    telemetry.addData("VuMark", "not visible");
                }
                */
            telemetry.update();


            if (imageReady())
            {

                telemetry.addData("Width", width);
                telemetry.addData("Height", height);

                int redValueLeft = -76800;
                int blueValueLeft = -76800;
                int greenValueLeft = -76800;

                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);
                //The last value must correspond to the downsampling value from above
                //rgbImage = ((FtcRobotControllerActivity) hardwareMap.appContext).captureScreenshot();

                telemetry.addData("Width", rgbImage.getWidth());
                telemetry.addData("Height", rgbImage.getHeight());
                telemetry.update();


                for (int x = 240; x < 480; x++)
                {
                    for (int y = 568; y < 854; y++)
                    {
                        if (x == 0 && y >= 568)
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x >= 0 && y == 568)
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x == 240 && y >= 568)
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                        if (x >= 240 && y == 853)
                        {
                            rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                        }
                    }
                }

                SaveImage(rgbImage);

                //Analyzing Jewel Color
                for (int x = 240; x < 480; x++)
                {
                    for (int y = 568; y < 854; y++)
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
                } else if (jewelColorInt == 1)
                {
                    telemetry.addData("Jewel Color", "1 : Blue");
                } else if (jewelColorInt == 2)
                {
                    telemetry.addData("Jewel Color", "Green? What Did You Do? Green Shouldn't Even Be An Option!");
                } else
                {
                    telemetry.addData("Jewel Color", "Something's Wrong");
                }
                telemetry.update();
            }


            }
            stopCamera();
            telemetry.addData("Not available?", "??");

        }

        //}
    /*String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    */
    }
}
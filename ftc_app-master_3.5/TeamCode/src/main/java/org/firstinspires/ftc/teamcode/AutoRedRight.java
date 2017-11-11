package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by User on 10/14/2017.
 */

@Autonomous (name = "AutoRedRight", group = "Auto")
public class AutoRedRight extends LinearOpModeCamera
{
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive();

    ServoMove servoMove = new ServoMove();

    String color = "undecided";

    double wristInit = 0.375;
    double knockInit = 0.928;
    double clawInit = 0.93;

    int jewelColorInt;

    @Override
    public void runOpMode()
    {

        robot.init(hardwareMap);

        robot.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor[] motors = new DcMotor[4];
        motors[0] = robot.motorRF;
        motors[1] = robot.motorRB;
        motors[2] = robot.motorLB;
        motors[3] = robot.motorLF;

        Servo[] servos = new Servo[2];
        servos[0] = robot.flapper;
        servos[1] = robot.slapper;

        robot.slapper.setPosition(0.8);
        robot.flapper.setPosition(1.0);
        robot.wrist.setPosition(wristInit);
        robot.knock.setPosition(knockInit);
        robot.claw.setPosition(clawInit);

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

        drive.timeDrive(750, 0.4, driveStyle.STRAFE_LEFT, motors);
            sleep(1000);
            drive.timeDrive(800, 0.5, driveStyle.STRAFE_RIGHT, motors);
        /*
        robot.slapper.setPosition(0.5);
        sleep(1000);
        robot.flapper.setPosition(0.7);
        sleep(1000);
        color = "red";
        if(color == "blue")
        {
            robot.slapper.setPosition(0.25);
        }
        if(color == "red")
        {
            robot.slapper.setPosition(0.75);
        }
        sleep(1000);
        robot.slapper.setPosition(0.5);
        sleep(1000);
        robot.flapper.setPosition(1.0);
        sleep(1000);
        robot.slapper.setPosition(1.0);
        sleep(1000);
        */
        servoMove.knockOffJewel(servos, jewelColorInt, "red");
            drive.timeDrive(85, 0.4, driveStyle.STRAFE_LEFT, motors);
            sleep(1000);
        drive.timeDrive(800, 0.5, driveStyle.BACKWARD, motors);
        sleep(1000);
        drive.timeDrive(750, 0.5, driveStyle.STRAFE_RIGHT, motors);
    }
}
}



package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link AccelerometerTest} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "Sensor: BNO055 IMU", group = "Sensor")

public class AccelerometerTest extends LinearOpMode
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    RobotHardware curiosity = new RobotHardware();

    Drive drive = new Drive();

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Acceleration accel;
    Position pos = new Position(DistanceUnit.INCH, 0, 0, 0, 0);

    double temperature;
    boolean stopMotors = true;
    double previousAccelX = 0;
    double previousAccelY = 0;
    double previousAccelZ = 0;

    double totalAccelX = 0;
    double totalAccelY = 0;
    double totalAccelZ = 0;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json."; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.temperatureUnit     = BNO055IMU.TempUnit.CELSIUS;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        curiosity.init(hardwareMap);

        DcMotor[] motors = new DcMotor[4];
        motors[0] = curiosity.motorRF;
        motors[1] = curiosity.motorRB;
        motors[2] = curiosity.motorLB;
        motors[3] = curiosity.motorLF;
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            if(!stopMotors) {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                drive.pivotRight(0.4, motors);
                while (Math.abs(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) < 84 && opModeIsActive()) {
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Heading Raw", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
                    //
                    telemetry.update();
                }
                curiosity.motorLB.setPower(0.0);
                curiosity.motorRB.setPower(0.0);
                curiosity.motorRF.setPower(0.0);
                curiosity.motorLF.setPower(0.0);
                stopMotors = true;
            }

            while(!imu.isAccelerometerCalibrated())
            {
                telemetry.addData("Accel Calibrating...","");
                telemetry.update();
                sleep(1000);
                //
            }
            //acceleration = imu.getAcceleration();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            accel = imu.getAcceleration();


            //totalAccelX += accel.xAccel;
            totalAccelY += accel.yAccel;
            //totalAccelZ += accel.zAccel;

            /*telemetry.addData("Heading Raw", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Position Z", pos.z);
            telemetry.addData("Position X", pos.x);
            telemetry.addData("Position Y", pos.y);
            */
            /*telemetry.addData("X Acceleration", (double)Math.round(accel.xAccel * 100d) / 100d);
            telemetry.addData("Y Acceleration", (double)Math.round(accel.yAccel * 100d) / 100d);
            telemetry.addData("Z Acceleration", (double)Math.round(accel.zAccel * 100d) / 100d);
            */

            //telemetry.addData("Total Accel X", (double)Math.round(totalAccelX * 100d)/ 100d);
            telemetry.addData("Total Accel Y", (double)Math.round(totalAccelY * 100d) / 100d);
            telemetry.addData("Y Accel", (double)Math.round(accel.yAccel * 1000d) / 1000d);
            //telemetry.addData("Total Accel Z", (double)Math.round(totalAccelZ * 100d)/ 100d);

            telemetry.update();

        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    /*void composeTelemetry() {

            // At the beginning of each telemetry update, grab a bunch of data
            // from the IMU that we will then display in separate lines.
            telemetry.addAction(new Runnable() { @Override public void run()
            {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                temperature = imu.getTemperature().temperature;

            }
            });


            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override public String value() {
                            return gravity.toString();
                        }
                    })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
        telemetry.addData("Temperature", temperature);
    }
    */


    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    /*String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    */
}

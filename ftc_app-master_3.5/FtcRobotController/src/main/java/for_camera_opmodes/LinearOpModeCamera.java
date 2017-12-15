package for_camera_opmodes;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.ImageFormat;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.os.Environment;
import android.util.Log;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class LinearOpModeCamera extends LinearOpMode
{

    public Camera camera;
    public CameraPreview preview;

    public int width;
    public int height;
    public YuvImage yuvImage = null;

    volatile private boolean imageReady = false;

    private int looped = 0;
    private String data;
    private int ds = 1; // downsampling parameter

    @Override
    // should be overwritten by extension class
    public void runOpMode() throws InterruptedException {

    }


    public Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera) {
            try {
                Camera.Parameters parameters = camera.getParameters();
                width = parameters.getPreviewSize().width;
                height = parameters.getPreviewSize().height;
                yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
                imageReady = true;
                looped += 1;
            } catch (Exception e) {

            }
        }
    };

    public void setCameraDownsampling(int downSampling) {
        ds = downSampling;
    }

    public boolean imageReady() {
        return imageReady;
    }

    public boolean isCameraAvailable() {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            Camera.getCameraInfo(i, info);
            if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
                cameraId = i;
                break;
            }
        }
        try {
            cam = Camera.open(cameraId);
        } catch (Exception e) {
            Log.e("Error", "Camera Not Available!");
            return false;
        }
        cam.release();
        cam = null;
        return true;
    }

    public Camera openCamera(int cameraInfoType) {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            Camera.getCameraInfo(i, info);
            if (info.facing == cameraInfoType) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
                cameraId = i;
                break;
            }
        }
        try {
            cam = Camera.open(cameraId);
        } catch (Exception e) {
            Log.e("Error", "Can't Open Camera");
        }
        return cam;
    }

    public void startCamera() {

        camera = openCamera(Camera.CameraInfo.CAMERA_FACING_FRONT);
        camera.setPreviewCallback(previewCallback);

        Camera.Parameters parameters = camera.getParameters();

        width = parameters.getPreviewSize().width / ds;
        height = parameters.getPreviewSize().height / ds;
        parameters.setPreviewSize(width, height);

        camera.setParameters(parameters);

        data = parameters.flatten();

        if (preview == null) {
           ((FtcRobotControllerActivity) hardwareMap.appContext).initPreviewLinear(camera, this, previewCallback);
        }
    }

    public void stopCameraInSecs(int duration) {
        Thread cameraKillThread = new Thread(new CameraKillThread(duration));

        cameraKillThread.start();
    }

    public class CameraKillThread implements Runnable {
        int dur;

        public CameraKillThread(int duration) {
            dur = duration;
        }

        public void run() {
            try {
                Thread.sleep(dur * 1000, 0);
            } catch (InterruptedException ex) {

            }

            stopCamera();
            imageReady = false;
        }
    }

    public void stopCamera() {
        if (camera != null) {
            if (preview != null) {
                ((FtcRobotControllerActivity) hardwareMap.appContext).removePreviewLinear(this);
                preview = null;
            }
            camera.stopPreview();
            camera.setPreviewCallback(null);
            if (camera != null) {
                camera.release();
            }
            camera = null;
        }
    }

    static public int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    static public int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    static public int blue(int pixel) {
        return pixel & 0xff;
    }

    static public int gray(int pixel) {
        return (red(pixel) + green(pixel) + blue(pixel));
    }

    static public int highestColor(int red, int green, int blue) {
        int[] color = {red, green, blue};
        int value = 0;
        for (int i = 1; i < 3; i++) {
            if (color[value] < color[i]) {
                value = i;
            }
        }
        return value;
    }

    // returns ROTATED image, to match preview window
    static public Bitmap convertYuvImageToRgb(YuvImage yuvImage, int width, int height, int downSample) {
        return OpModeCamera.convertYuvImageToRgb(yuvImage, width, height, downSample);
    }

    public int normalizePixels(int value) {
        value /= 100000;
        return value;
    }

    public void SaveImage(Bitmap finalBitmap) {

        String root = Environment.getExternalStorageDirectory().toString();
        File myDir = new File(root);
        myDir.mkdirs();
        SimpleDateFormat s = new SimpleDateFormat("dd MM yyyy hhmmss");
        String format = s.format(new Date());
        String fname = format +".jpg";
        File file = new File (myDir, fname);
        if (file.exists ()) file.delete ();
        try {
            FileOutputStream out = new FileOutputStream(file);
            finalBitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
            out.flush();
            out.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void SaveRGB (Bitmap colorBitmap)
    {
        String pixelColorMap = "";
        String rtemp;
        String gtemp;
        String btemp;
        String xv;
        String yv;
        int pixelPlaceholder;
        int Rtemp;
        int Gtemp;
        int Btemp;
        for(int y = 0; y < 640; y++)
        {
            for(int x = 0; x < 480; x++)
            {
                pixelPlaceholder = colorBitmap.getPixel(x,y);
                xv = Integer.toString(x);
                yv = Integer.toString(y);
                Rtemp = red(pixelPlaceholder);
                rtemp = Integer.toString(Rtemp);
                Gtemp = green(pixelPlaceholder);
                gtemp = Integer.toString(Gtemp);
                Btemp = blue(pixelPlaceholder);
                btemp = Integer.toString(Btemp);

                pixelColorMap = pixelColorMap + x + "," + y + " - ";
                pixelColorMap = pixelColorMap + rtemp + ",";
                pixelColorMap = pixelColorMap + gtemp + ",";
                pixelColorMap = pixelColorMap + btemp + System.getProperty("line.separator");
            }
        }
    }
    public int FindJewelsCenter (Bitmap cameraBitmap)
    {
        int width1 = cameraBitmap.getWidth();
        int lineRightX = -1;
        int tempPixel;
        int white = 120;
        int x = width1 - 1;
        //Camera field of view should equal 960x1280 as long as downsampling is equal to 1

        while(lineRightX == -1 && x > 0)
        {
            for(x = width1 - 1; x > 0; x--)
            {
                tempPixel = cameraBitmap.getPixel(x,100);
                if(red(tempPixel) > white && blue(tempPixel) > white && green(tempPixel) > white)
                {
                    lineRightX = x;
                }
            }
        }
        return lineRightX;
    }

}
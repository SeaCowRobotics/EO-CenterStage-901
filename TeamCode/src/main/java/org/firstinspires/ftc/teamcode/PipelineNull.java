package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Example class that processes camera images with OpenCV module calls.
 * This code searches for gold/yellow colors and displays a rectangle around
 * the largest contour of the gold/yellow filtering.
 */
public class PipelineNull extends OpenCvPipeline {
    private final String TAG = this.getClass().getSimpleName();
    private Mat displayMat = new Mat(); // Image Mat to be displayed on screen
    public  Mat fileMat    = new Mat(); // Image Mat to hold image to write to a file

    /**
     * Method declared in OpenCVAgent
     * Put all code to process camera frames into this method.
     */
    @Override
    public Mat processFrame(Mat input) {
        
        Log.d(TAG, "processing frame");

        Imgproc.cvtColor(input, fileMat, Imgproc.COLOR_BGR2RGB);;

        input.copyTo(displayMat);

        return displayMat;
    }

}
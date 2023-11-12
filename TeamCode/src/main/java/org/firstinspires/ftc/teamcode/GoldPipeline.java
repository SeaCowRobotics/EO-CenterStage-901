/*
 * Copyright (c) 2019 Zed Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Example class that processes camera images with OpenCV module calls.
 * This code searches for gold/yellow colors and displays a rectangle around
 * the largest contour of the gold/yellow filtering.
 */
public class GoldPipeline extends OpenCvPipeline {

    private static final String TAG = "Gold Detector"; // Logging ID tag

    private Mat displayMat = new Mat(); // Image Mat to be displayed on screen
    private Mat rgbMat     = new Mat(); // Image Mat to be mathematically processed
    private Mat hsvMat     = new Mat(); // Image Mat returned by RGB->HSV method
    private Mat blurredMat = new Mat(); // Image Mat returned by blurring method
    private Mat goldMat    = new Mat(); // Image Mat returned by gold color filter
    public  Mat fileMat    = new Mat(); // Image Mat to hold image to write to a file

    private Scalar rgbRed  = new Scalar(255,0,0);  // RGB color set to red
    private Scalar rgbBlue = new Scalar(0,0,255);  // RGB color set to blue
    private Size blurSize  = new Size(3,3);        // kernel size of dimensions 3x3
    public  List<MatOfPoint> contourList = new ArrayList<>(); // List of contours
    private Mat hierarchy  = new Mat(); // placeholder for contour hierarchy

    private boolean found = false;

    boolean viewportPaused;

    public boolean isFound() {
        return found;
    }

    /**
     * Method declared in OpenCVAgent
     * Put all code to process camera frames into this method.
     */
    @Override
    public Mat processFrame(Mat input) {
        
        Log.d(TAG, "processing frame");

        // copy rgba Mat image for output display
        // we will continue processing on rgba Mat
        input.copyTo(displayMat);
        input.copyTo(rgbMat);

        // Convert the rgb image from rgb colorspace to hsv colorspace
        //  args: (src, dst, code)
        //         code: COLOR_RGB2HSV_FULL
        Imgproc.cvtColor(rgbMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        
        // Fuzz out spurious pixels with blurring operation
        //   args: (src, dst, ksize, -)
        Imgproc.GaussianBlur(hsvMat, blurredMat, blurSize, 0);
        
        // Generate a masked Mat, given lower and upper bounds for color filtering
        // Given bounds for Gold Mineral detection:
        Scalar lowerHSVbound = new Scalar(25, 100, 100);
        Scalar upperHSVbound = new Scalar(45, 255, 255);
        // Use OpenCV method to generate mask; sets output to 255 if within bounds, else 0.
        //  args: (src, lowerb, upperb, dst)
        //         lowerb, upperb are Scalar in given colorspace
        Core.inRange(blurredMat, lowerHSVbound, upperHSVbound, goldMat);
                
        // Use OpenCV to find all contours in gold-masked image
        // args: (src, <MatOfPoint> contours, hierarchy, mode, method)
        //        mode:   RETR_LIST - retrieve all contours without a hierarchy
        //        method: CHAIN_APPROX_SIMPLE - compress segments, minimizes number of contour points
        contourList = new ArrayList<>();
        Imgproc.findContours(goldMat, contourList, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        
        // Draw the contours onto the display image
        // args: (src, <MatOfPoint> contours, contourIdx, color, thickness)
        //       contourIdx: -1 draw all contours
        //       color:      Scalar RGB color
        //       thickness:  How many pixels thick to draw the contours
        Imgproc.drawContours(displayMat,contourList,-1,rgbRed,2);

        // Loop through all contours to find the contour with the largest area
        double biggestArea = 0;
        MatOfPoint biggestContour = null;
        for(MatOfPoint contour : contourList){
            double area = Imgproc.contourArea(contour);
            if (area > biggestArea) {
                biggestContour = contour;
            }
        }

        // If there was a largest contour found then
        // draw bounding rectangle onto display image around the largest contour
        if (biggestContour != null) {
            if (Imgproc.contourArea(biggestContour) > 100) {
                Rect rect = Imgproc.boundingRect(biggestContour);
                Imgproc.rectangle(displayMat, rect.tl(), rect.br(), rgbBlue, 2);
                // make a copy to save to file
                Imgproc.cvtColor(displayMat, fileMat, Imgproc.COLOR_BGR2RGB);
                found = true;
            }

        }

        return displayMat;
    }

}
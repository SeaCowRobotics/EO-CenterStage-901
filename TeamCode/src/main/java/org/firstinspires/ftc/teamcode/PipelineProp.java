package org.firstinspires.ftc.teamcode;

import android.util.Log;
import org.openftc.easyopencv.OpenCvPipeline;

//requires:
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


/**
 * Example class that processes camera images with OpenCV module calls.
 * This code searches for gold/yellow colors and displays a rectangle around
 * the largest contour of the gold/yellow filtering.
 */
public class PipelineProp extends OpenCvPipeline {
    private final String TAG = this.getClass().getSimpleName();
    private Mat displayMat = new Mat(); // Image Mat to be displayed on screen
    public  Mat fileMat    = new Mat(); // Image Mat to hold image to write to a file

    public int foundPropHere = 0;
    public Point leftPoint;
    public Point centerPoint;
    public Scalar leftColor;
    public Scalar centerColor;

    public Mat doLinkCropImage_blue(Mat matImgSrc) {
        int mark;
        Scalar blank = new Scalar(0,0,0);
        Mat matImgDst = Mat.zeros(matImgSrc.rows(), matImgSrc.cols(), CvType.CV_8U);
        matImgSrc.copyTo(matImgDst);
        mark = 0;
        if (mark > 0) {
            Mat leftMat = matImgDst.submat(new Rect(0,0, mark, matImgSrc.rows()));
            leftMat.setTo(blank);
        }
        mark = 320;
        if (mark < matImgSrc.cols()) {
            Mat rightMat = matImgDst.submat(new Rect(mark, 0, matImgSrc.cols()-mark-1, matImgSrc.rows()));
            rightMat.setTo(blank);
        }
        mark = 133;
        if (mark > 0) {
            Mat topMat = matImgDst.submat(new Rect(0, 0, matImgSrc.cols(), mark));
            topMat.setTo(blank);
        }
        mark = 240;
        if (mark < matImgSrc.rows()) {
            Mat bottomMat = matImgDst.submat(new Rect(0, mark, matImgSrc.cols(), matImgSrc.rows()-mark-1));
            bottomMat.setTo(blank);
        }
        return matImgDst;
    }

    public Mat doLinkInRangeHSV_blue(Mat matImgSrc) {
        Mat src = new Mat();
        Mat msk = new Mat();
        // Initialize output Mat to all zeros; and to same Size as input Mat
        Mat matImgDst = Mat.zeros(
                matImgSrc.rows(), // int - number of rows
                matImgSrc.cols(), // int - number of columns
                CvType.CV_8U      // int - Mat data type
        );
        // If the source image was a file then the Mat is BGR (as this code assumes)
        // BUT if the source image was a camera then the Mat is likely RGB, so instead use COLOR_RGB2HSV
        // Convert source Mat in BGR color space to HSV color space
        Imgproc.cvtColor(
                matImgSrc,             // Mat - source
                src,                   // Mat - destination
                Imgproc.COLOR_BGR2HSV  // int - code space conversion code
        );
        // Create masking Mat msk of all pixels within Scalar boundaries
        Scalar lowerb = new Scalar(87, 37, 50);
        Scalar upperb = new Scalar(131, 255, 255);
        Core.inRange(
                src,       // Mat    - input Mat
                lowerb,    // Scalar - inclusive lower boundary scalar
                upperb,    // Scalar - inclusive upper boundary scalar
                msk        // Mat    - output Mat, same size as src, and of CV_8U type
        );
        // Copy matImgSrc pixels to matImgDst, filtered by msk
        Core.copyTo(
                matImgSrc,  // Mat - source Mat
                matImgDst,  // Mat - destination Mat
                msk         // Mat - masking Mat
        );
        return matImgDst;
    }

    public List<MatOfPoint> doLinkFindContours_blue(Mat matImgSrc) {
        Mat gray = new Mat();
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        // Convert source Mat in BGR color space to Gray color space
        Imgproc.cvtColor(
                matImgSrc,              // Mat - source
                gray,                   // Mat - destination
                Imgproc.COLOR_BGR2GRAY  // int - code space conversion code
        );
        Imgproc.findContours(
                gray,         // Mat - input image
                contours,     // List of MatOfPoints - output List of contours
                hierarchy,    // Mat - output hierarchy Mat
                Imgproc.RETR_TREE,    // int - contour retrieval mode
                Imgproc.CHAIN_APPROX_SIMPLE    // int - contour approximation method
        );
        return contours;
    }
    public List<MatOfPoint> doLinkFilterContours_blue(List<MatOfPoint> contours) {
        List<MatOfPoint> filteredContours = new ArrayList<>();
        double area;
        double perimeter;
        MatOfPoint2f contour2f;
        for (int i = 0; i < contours.size(); i++) {
            // calculate area and perimeter of each contour
            area = Imgproc.contourArea(contours.get(i),false);
            contour2f = new MatOfPoint2f(contours.get(i).toArray());
            perimeter = Imgproc.arcLength(contour2f,true);
            // only add contours within desired area and perimeter to list of filtered contours
            if (( area > 100.0 ) &&
                    (area < 10000.0) &&
                    ( perimeter > -1.0) &&
                    (perimeter < Double.POSITIVE_INFINITY)) {
                filteredContours.add(contours.get(i));
            }
        }
        return filteredContours;
    }
    public int doLinkCenterStageProp_blue(List<MatOfPoint> contours) {
        int leftSpikeMin = 0;
        int leftSpikeMax = 10;
        int leftPropMin  = 100;
        int leftPropMax  = 10000;
        int centerSpikeMin = 0;
        int centerSpikeMax = 10;
        int centerPropMin  = 100;
        int centerPropMax  = 10000;
        List<Point>   circleCenters = new ArrayList<>();
        List<Double>  contourAreas = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            Point center = new Point();
            float[] radius = new float[1];
            MatOfPoint   contour = contours.get(i);
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            Imgproc.minEnclosingCircle(
                    contour2f, // MatOfPoint2f - input mat of points,
                    center,    // Point        - output center,
                    radius     // float[]      - output radius
            );
            circleCenters.add(center);
            contourAreas.add(Imgproc.contourArea(contour, false));
        }
        int leftIdx = -1;
        int centerIdx = -1;
        int propIndex = 0;
        boolean leftIsProp = false;
        boolean leftIsSpike = false;
        boolean centerIsProp = false;
        boolean centerIsSpike = false;
        int area;
        if (circleCenters.size() != 2) {
            return propIndex;
        }
        if (circleCenters.get(0).x < circleCenters.get(1).x) {
            leftIdx = 0;
            centerIdx = 1;
        } else {
            leftIdx = 1;
            centerIdx = 0;
        }
        area = contourAreas.get(leftIdx).intValue();
        leftIsSpike = ((area >= leftSpikeMin) && (area <= leftSpikeMax));
        leftIsProp  = ((area >= leftPropMin) && (area <= leftPropMax));
        area = contourAreas.get(centerIdx).intValue();
        centerIsSpike = ((area >= centerSpikeMin) && (area <= centerSpikeMax));
        centerIsProp  = ((area >= centerPropMin) && (area <= centerPropMax));
        if (leftIsSpike && centerIsSpike) {
            propIndex = 3;
        } else if (leftIsProp && centerIsSpike) {
            propIndex = 1;
        } else if (leftIsSpike && centerIsProp) {
            propIndex = 2;
        }

        if (propIndex > 0) {
            leftPoint = new Point(circleCenters.get(leftIdx).x, circleCenters.get(leftIdx).y);
            centerPoint = new Point(circleCenters.get(centerIdx).x, circleCenters.get(centerIdx).y);
        }
        if (leftIsSpike) {
            leftColor = new Scalar(255.0, 255.0, 0.0);
        } else {
            leftColor = new Scalar(0.0, 255.0, 0.0);
        }
        if (centerIsSpike) {
            centerColor = new Scalar(255.0, 255.0, 0.0);
        } else {
            centerColor = new Scalar(0.0, 255.0, 0.0);
        }

        return propIndex;
    }

    public int doChain_blue(Mat matImgSrc) {
        Mat cropImageMat = doLinkCropImage_blue(matImgSrc);
        Mat inRangeHSVMat = doLinkInRangeHSV_blue(cropImageMat);
        List<MatOfPoint> findContoursList = doLinkFindContours_blue(inRangeHSVMat);
        List<MatOfPoint> filterContoursList = doLinkFilterContours_blue(findContoursList);
        int centerStageProp = doLinkCenterStageProp_blue(filterContoursList);
        return centerStageProp;
    }



    /**
     * Method declared in OpenCVAgent
     * Put all code to process camera frames into this method.
     */
    @Override
    public Mat processFrame(Mat input) {
        
        Log.d(TAG, "processing frame");

        foundPropHere = doChain_blue(input);

        Imgproc.cvtColor(input, fileMat, Imgproc.COLOR_BGR2RGB);;

        input.copyTo(displayMat);

        return displayMat;

    }

}
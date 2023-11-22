/*
 * Copyright (c) 2019 OpenFTC Team
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp
public class OpModeDetectPropRed extends LinearOpMode
{
    OpenCvWebcam webcam;

    PipelinePropRed pipeline;

    int propIsHere = 0;
    String propStr = "";

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new PipelinePropRed();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive()) {
            if (propIsHere == 0) {

                telemetry.addData("Looking for Prop", "");
                telemetry.update();

                int foundPropHere = pipeline.foundPropHere;
                if (foundPropHere > 0) {
                    propIsHere = foundPropHere;
                    switch (propIsHere) {
                        case 1: propStr = "LEFT"; break;
                        case 2: propStr = "CENTER"; break;
                        case 3: propStr = "RIGHT"; break;
                        default: propStr = "WHAT??"; break;
                    }
                    webcam.stopStreaming();
                    // put left, center circles into saved image
                    Imgproc.circle(
                            pipeline.fileMat,       // Mat img - input/output image
                            pipeline.leftPoint,     // Point center
                            3,                      // int radius
                            pipeline.leftColor,     // Scalar color
                            3                       // int thickness
                    );
                    Imgproc.circle(
                            pipeline.fileMat,       // Mat img - input/output image
                            pipeline.centerPoint,   // Point center
                            3,                      // int radius
                            pipeline.centerColor,   // Scalar color
                            3                       // int thickness
                    );
                    SimpleDateFormat dateFormat = new SimpleDateFormat ("yyyy-MM-dd@HH-mm-ss", Locale.US);
                    Imgcodecs.imwrite("/sdcard/FIRST/" + "image_" + dateFormat.format (new Date())+".jpg", pipeline.fileMat);
                }

            } else {

                telemetry.addData("Prop is ", propStr);
                telemetry.update();

            }
            sleep(50);
        } // while opModeIsActive
    }

}
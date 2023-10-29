package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.objdetect.QRCodeDetector;

import java.util.ArrayList;

public class SignalDetectionPipeline extends OpenCvPipeline {

    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    ArrayList<Mat> yCrCbChannels = new ArrayList(3);
    int counter; // have to give an initial value


    void inputToY(Mat input) {

    }


    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        if (Constants.signalDetectionMethod == 1) {

        } else if (Constants.signalDetectionMethod == 2) {

        }

        return input;
    }

    public int getCounter() {
        return counter;
    }

}


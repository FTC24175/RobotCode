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

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToY(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);
    }

    int detectBlackWhite(Mat input) {
        int edgeCounter = 0;
        int lEdgeCounter = 0;
        int rEdgeCounter = 0;
        int x1 = Constants.leftBoundary;
        int x2 = Constants.rightBoundary;
        int midy = Constants.middleLine;
        for (int j = x1 + 1; j < x2; j++) {
//            if (input.at(Byte.class, (y2 + y1) / 2, j).getV4c().get_0() - input.at(Byte.class, (y2 + y1) / 2, j - 1).getV4c().get_0() > Constants.changeThresh) {
            if ((input.at(Byte.class, midy, j).getV().byteValue() - input.at(Byte.class, midy, j - 1).getV().byteValue()) > Constants.changeThresh) {
                lEdgeCounter += 1;
            }
            if ((input.at(Byte.class, midy, j - 1).getV().byteValue() - input.at(Byte.class, midy, j).getV().byteValue()) > Constants.changeThresh) {
                rEdgeCounter += 1;
            }
        }
        edgeCounter = Math.min(lEdgeCounter, rEdgeCounter);
        if (edgeCounter == 0) { // clip
            edgeCounter = 1;
        } else if (edgeCounter > 3) {
            edgeCounter = 3;
        }
        return edgeCounter;
    }

    int detectLinesVH(Mat input) {
        int horCounter = 0;
        int verCounter = 0;
        int x1 = Constants.HVLeftBoundary;
        int x2 = Constants.HVRightBoundary;
        int midx = (Constants.HVLeftBoundary + Constants.HVRightBoundary) / 2;
        int y1 = Constants.HVTopBoundary;
        int y2 = Constants.HVBottomBoundary;
        int midy = (Constants.HVTopBoundary + Constants.HVBottomBoundary) / 2;

        for (int j = x1 + 1; j < x2; j++) {
            if ((input.at(Byte.class, midy, j).getV().byteValue() - input.at(Byte.class, midy, j - 1).getV().byteValue()) > Constants.changeThresh) {
                horCounter += 1;
            }
        }

        for (int j = y1 + 1; j < y2; j++) {
            if ((input.at(Byte.class, j, midx).getV().byteValue() - input.at(Byte.class, j - 1, midx).getV().byteValue()) > Constants.changeThresh) {
                verCounter += 1;
            }
        }

        if (horCounter >= 5 && verCounter <= 2) {
            return 3; // Vertical bars
        } else if (horCounter <= 2 && verCounter >= 5) {
            return 2; // Horizontal bars
        } else if (horCounter <= 2 && verCounter <= 2) {
            return 1; // No bars
        } else {
            return 2; // Default is Horizontal bar
        }
    }

    int detectLinesVHD(Mat input) {
        int horCounter = 0;
        int verCounter = 0;
        int x1 = Constants.HVLeftBoundary;
        int x2 = Constants.HVRightBoundary;
        int midx = (Constants.HVLeftBoundary + Constants.HVRightBoundary) / 2;
        int y1 = Constants.HVTopBoundary;
        int y2 = Constants.HVBottomBoundary;
        int midy = (Constants.HVTopBoundary + Constants.HVBottomBoundary) / 2;

        for (int j = x1 + 1; j < x2; j++) {
            if ((input.at(Byte.class, midy, j).getV().byteValue() - input.at(Byte.class, midy, j - 1).getV().byteValue()) > Constants.changeThresh) {
                horCounter += 1;
            }
        }

        for (int j = y1 + 1; j < y2; j++) {
            if ((input.at(Byte.class, j, midx).getV().byteValue() - input.at(Byte.class, j - 1, midx).getV().byteValue()) > Constants.changeThresh) {
                verCounter += 1;
            }
        }

        if (horCounter >= 5 && verCounter <= 2) {
            return 3; // Vertical bars
        } else if (horCounter <= 2 && verCounter >= 5) {
            return 2; // Horizontal bars
        } else if (horCounter >= 5 && verCounter >= 5) {
            return 1; // Diagonal bars
        } else {
            return 2; // Default is Horizontal bar
        }
    }

    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        if (Constants.signalDetectionMethod == 1) {
            Rect ROIRect = new Rect(700, 200, 500, 700);
            Mat croppedInput = input.submat(ROIRect);
            QRCodeDetector QRReader = new QRCodeDetector();
            //  String QRString = QRReader.detectAndDecodeCurved(croppedInput);
            String QRString = QRReader.detectAndDecode(croppedInput);
            if (!QRString.isEmpty()) {
                counter = Integer.parseInt(QRString);
            }
            Imgproc.rectangle( // rings
                    input, // Buffer to draw on
                    new Point(700, 200), // First point which defines the rectangle
                    new Point(1200, 900), // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
        } else if (Constants.signalDetectionMethod == 2) {
            inputToY(input);
            Y.copyTo(input);
            counter = detectBlackWhite(input);
            YCrCb.release(); // don't leak memory!
            Y.release(); // don't leak memory!
            for (Mat colChannel:yCrCbChannels) {
                colChannel.release();
            }

            //line detection rect
            Imgproc.rectangle( // rings
                    input, // Buffer to draw on
                    new Point(Constants.leftBoundary, Constants.middleLine - 20), // First point which defines the rectangle
                    new Point(Constants.rightBoundary, Constants.middleLine + 20), // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.line(input, new Point(Constants.leftBoundary, Constants.middleLine), new Point(Constants.rightBoundary, Constants.middleLine), new Scalar(0, 0, 255), 3);
        } else if (Constants.signalDetectionMethod == 3) {
            inputToY(input);
            Y.copyTo(input);
            counter = detectLinesVH(input);
            YCrCb.release(); // don't leak memory!
            Y.release(); // don't leak memory!
            for (Mat colChannel:yCrCbChannels) {
                colChannel.release();
            }

            //line detection rect
            Imgproc.rectangle( // rings
                    input, // Buffer to draw on
                    new Point(Constants.HVLeftBoundary, Constants.HVTopBoundary), // First point which defines the rectangle
                    new Point(Constants.HVRightBoundary, Constants.HVBottomBoundary), // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.line(input, new Point(Constants.HVLeftBoundary, (Constants.HVTopBoundary + Constants.HVBottomBoundary) / 2), new Point(Constants.HVRightBoundary, (Constants.HVTopBoundary + Constants.HVBottomBoundary) / 2), new Scalar(0, 0, 255), 3);
            Imgproc.line(input, new Point((Constants.HVLeftBoundary + Constants.HVRightBoundary) / 2, Constants.HVTopBoundary), new Point( (Constants.HVLeftBoundary + Constants.HVRightBoundary) / 2, Constants.HVBottomBoundary), new Scalar(0, 0, 255), 3);
        } else if (Constants.signalDetectionMethod == 4) {
            inputToY(input);
            Y.copyTo(input);
            counter = detectLinesVHD(input);
            YCrCb.release(); // don't leak memory!
            Y.release(); // don't leak memory!
            for (Mat colChannel:yCrCbChannels) {
                colChannel.release();
            }

            //line detection rect
            Imgproc.rectangle( // rings
                    input, // Buffer to draw on
                    new Point(Constants.HVLeftBoundary, Constants.HVTopBoundary), // First point which defines the rectangle
                    new Point(Constants.HVRightBoundary, Constants.HVBottomBoundary), // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.line(input, new Point(Constants.HVLeftBoundary, (Constants.HVTopBoundary + Constants.HVBottomBoundary) / 2), new Point(Constants.HVRightBoundary, (Constants.HVTopBoundary + Constants.HVBottomBoundary) / 2), new Scalar(0, 0, 255), 3);
            Imgproc.line(input, new Point((Constants.HVLeftBoundary + Constants.HVRightBoundary) / 2, Constants.HVTopBoundary), new Point( (Constants.HVLeftBoundary + Constants.HVRightBoundary) / 2, Constants.HVBottomBoundary), new Scalar(0, 0, 255), 3);
        }

        return input;
    }

    public int getCounter() {
        return counter;
    }

}


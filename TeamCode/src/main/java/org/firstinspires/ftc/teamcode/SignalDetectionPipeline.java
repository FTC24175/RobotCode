package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.calcHist;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    int counter = 0; // have to give an initial value

    private Mat workingMatrix = new Mat();
    private Mat imageLeft = new Mat();
    private Mat imageCenter = new Mat();
    private Mat imageRight = new Mat();

    double leftTotal, centerTotal, rightTotal;
    double meanLeft, meanCenter, meanRight;

    Telemetry telemetry;

    Scalar meanImage;

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


    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        if (Constants.signalDetectionMethod == 1) {
            input.copyTo(workingMatrix);
            if (workingMatrix.empty()) {
                return input;
            }

            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);




            imageLeft = workingMatrix.submat(300, 700, 10, 400);
            imageCenter = workingMatrix.submat(30, 250, 220, 700);
            imageRight = workingMatrix.submat(162, 425, 853, 1066);

            leftTotal = Core.sumElems(imageLeft).val[1];
            centerTotal = Core.sumElems(imageCenter).val[1];
            rightTotal = Core.sumElems(imageRight).val[1];

            meanLeft = leftTotal/(401*390);
            meanCenter = centerTotal/(221*481);
            meanRight = rightTotal/(264*214);

            int countl = 0;
            int countc = 0;
            int countr = 0;
            int colorThreshold = 100;


            for (int i = 300; i < 700; i+=5) {
                for (int j = 10; j < 400; j+=5) {
                    double[] a = workingMatrix.get(i, j);
                    if (a[1] > colorThreshold) {
                        countl++;
                    }

                }
            }

            for (int i = 30; i < 250; i+=5) {
                for (int j = 220; j < 700; j+=5) {
                    double[] a = workingMatrix.get(i, j);
                    if (a[1] > colorThreshold) {
                        countc++;
                    }

                }
            }

            for (int i = 162; i < 425; i+=5) {
                for (int j = 853; j < 1066; j+=5) {
                    double[] a = workingMatrix.get(i, j);
                    if (a[1] > colorThreshold) {
                        countr++;
                    }

                }
            }

            meanCenter = countc;
            meanLeft = countl;
            meanRight = countr;

            if (countl >= countc) {
                if (countl >= countr) {
                    counter = countl;
                } else {
                    counter = countr;
                }
            } else {
                if (countc >= countr) {
                    counter = countc;
                } else {
                    counter = countr;
                }

            }

        }
        return input;

    }
    public int getCounter() {
        return counter;
    }

    public double getMeanCenter(){
        return meanCenter;
    }
    public double getMeanLeft(){
        return meanLeft;
    }

    public double getMeanRight(){
        return meanRight;
    }
}






package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SignalDetectionPipeline extends OpenCvPipeline {
    Scalar HOT_PINK = new Scalar(196, 23, 112);

    public Scalar scalarLowerYCrCb;// = new Scalar(0.0, 180.0, 60.0);
    public Scalar scalarUpperYCrCb;// = new Scalar(255.0, 255.0, 120.0);



    public volatile boolean error = false;
    public volatile Exception debug;

    private double borderLeftX;     //fraction of pixels from the left side of the cam to skip
    private double borderRightX;    //fraction of pixels from the right of the cam to skip
    private double borderTopY;      //fraction of pixels from the top of the cam to skip
    private double borderBottomY;   //fraction of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    private int loopCounter = 0;
    private int pLoopCounter = 0;

    private final Mat mat = new Mat();
    private final Mat processed = new Mat();

    private Rect maxRect = new Rect(600,1,1,1);

    private double maxArea = 0;
    private boolean first = false;

    private final Object sync = new Object();

    public SignalDetectionPipeline(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }
    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }
    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    public void configureBorders(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            synchronized (sync) {
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();

                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(areaPoints);

                        if (                        rect.area() > maxArea
                                && rect.x + (rect.width / 2.0)  > (borderLeftX * CAMERA_WIDTH)
                                && rect.x + (rect.width / 2.0)  < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)

                                || loopCounter - pLoopCounter   > 6
                                && rect.x + (rect.width / 2.0)  > (borderLeftX * CAMERA_WIDTH)
                                && rect.x + (rect.width / 2.0)  < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                        ){
                            maxArea = rect.area();
                            maxRect = rect;
                            pLoopCounter++;
                            loopCounter = pLoopCounter;
                            first = true;
                        }
                        else if(loopCounter - pLoopCounter > 10){
                            maxArea = new Rect().area();
                            maxRect = new Rect();
                        }

                        areaPoints.release();
                    }
                    contour.release();
                }
                if (contours.isEmpty()) {
                    maxRect = new Rect(600,1,1,1);
                }
            }
            if (first && maxRect.area() > 500) {
                Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2);
            }
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * CAMERA_WIDTH),
                    (int) (borderTopY * CAMERA_HEIGHT),
                    (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_WIDTH)),
                    (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT) - (borderTopY * CAMERA_HEIGHT))
            ), HOT_PINK, 2);

            Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);

            loopCounter++;
        } catch (Exception e) {
            debug = e;
            error = true;
        }
        return input;
    }

    public int getRectHeight() {
        synchronized (sync) {
            return maxRect.height;
        }
    }
    public int getRectWidth() {
        synchronized (sync) {
            return maxRect.width;
        }
    }
    public int getRectX() {
        synchronized (sync) {
            return maxRect.x;
        }
    }
    public int getRectY() {
        synchronized (sync) {
            return maxRect.y;
        }
    }
    public double getRectMidpointX() {
        synchronized (sync) {
            return getRectX() + (getRectWidth() / 2.0);
        }
    }
    public double getRectMidpointY() {
        synchronized (sync) {
            return getRectY() + (getRectHeight() / 2.0);
        }
    }
    public Point getRectMidpointXY() {
        synchronized (sync) {
            return new Point(getRectMidpointX(), getRectMidpointY());
        }
    }
    public double getAspectRatio() {
        synchronized (sync) {
            return getRectArea() / (CAMERA_HEIGHT * CAMERA_WIDTH);
        }
    }
    public double getRectArea() {
        synchronized (sync) {
            return maxRect.area();
        }
    }
}



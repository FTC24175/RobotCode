package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="cameraTest")
public class test extends LinearOpMode {


    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera

    private int leftPos;
    private int rightPos;
    public static Attachments iRobot = new Attachments();

    private ElapsedTime runtime = new ElapsedTime();

    OpenCvWebcam webcam;
    SignalDetectionPipeline signalDetectionPipeline;
    int signal = 0;

    double debugValue = 0.0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize all the parts of the robot
        iRobot.initialize(hardwareMap);

        iRobot.resetEncoder();
        iRobot.rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //waitForStart();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalDetectionPipeline = new SignalDetectionPipeline();
        webcam.setPipeline(signalDetectionPipeline); // start camera processFrame()
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start showing the camera
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //iRobot.setClawServo(Constants.clawClose);
        while (!isStarted()) {

            signal = signalDetectionPipeline.getCounter();
            telemetry.addData("Signal", signal);
            //telemetry.update();
            debugValue = signalDetectionPipeline.getMeanCenter();
            telemetry.addData("meanCenter", debugValue);
            debugValue = signalDetectionPipeline.getMeanLeft();
            telemetry.addData("meanLeft", debugValue);
            debugValue = signalDetectionPipeline.getMeanRight();
            telemetry.addData("meanRight", debugValue);
            telemetry.update();

        }
        runtime.reset();

        //encoderStraightDrive(1000, 1000, 0.5);

        if (signal == 1) {
            iRobot.forwardMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(1500, 1500, 0.3); // move forward
            iRobot.resetEncoder();
            iRobot.moveRobot(-550, 550, 0.3); // turn left
            iRobot.resetEncoder();
            iRobot.moveRobot(200, 200, 0.3); // move forwards a bit
            // reverse action
            iRobot.backMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(200, 200, 0.3); // move forwards a bit
            iRobot.resetEncoder();
            iRobot.moveRobot(-550, 550, 0.3); // move forwards a bit
            iRobot.resetEncoder();
            iRobot.moveRobot(1500, 1500, 0.3); // move forwards a bit



        } else if (signal == 2) {
            iRobot.forwardMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(1800, 1800, 0.3); // move forward
            //reverse action
            iRobot.backMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(1800, 1800, 0.3); // move forwards a bit
        } else{
            iRobot.forwardMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(1500, 1500, 0.3); // move forward
            iRobot.resetEncoder();
            iRobot.moveRobot(550, -550, 0.3); // turn left
            iRobot.resetEncoder();
            iRobot.moveRobot(200, 200, 0.3); // move forwards a bit
            //reverse action
            iRobot.backMode();
            iRobot.resetEncoder();
            iRobot.moveRobot(200, 200, 0.3); // move forwards a bit
            iRobot.resetEncoder();
            iRobot.moveRobot(550, -550, 0.3); // move forwards a bit
            iRobot.resetEncoder();
            iRobot.moveRobot(1500, 1500, 0.3); // move forwards a bit
        }


    }


}

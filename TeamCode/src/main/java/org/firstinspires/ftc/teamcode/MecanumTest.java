package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.*;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import static android.os.SystemClock.sleep;
import java.util.List;

@TeleOp(name="mecanumtest")
public class MecanumTest extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("motor1");
        backLeftMotor = hardwareMap.dcMotor.get("motor2");
        frontRightMotor = hardwareMap.dcMotor.get("motor3");
        backRightMotor = hardwareMap.dcMotor.get("motor4");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        List<AprilTagDetection> myAprilTagDetections;
        int myAprilTagIdCode;

        boolean aprilTagRunning = false;

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            //make sure ^^ is negated
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y,x);
            double power = Math.hypot(x,y);



            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin),
                    Math.abs(cos));
            double leftFront = power * cos/max + turn;
            double rightFront = power * sin/max - turn;
            double leftRear = power * sin/max + turn;
            double rightRear = power * cos/max - turn;

            if ((power + Math.abs(turn)) > 1) {
                leftFront /= power + Math.abs(turn);
                rightFront /= power + Math.abs(turn);
                leftFront /= power + Math.abs(turn);
                rightRear /= power + Math.abs(turn);
            }

            frontLeftMotor.setPower(leftFront);
            backLeftMotor.setPower(leftRear);
            frontRightMotor.setPower(rightFront);
            backRightMotor.setPower(rightRear);

            if(gamepad1.dpad_up) {
                frontLeftMotor.setPower(1);
                backLeftMotor.setPower(1);
                frontRightMotor.setPower(1);
                backRightMotor.setPower(1);
            }
            if(gamepad1.dpad_down) {
                frontLeftMotor.setPower(-1);
                backLeftMotor.setPower(-1);
                frontRightMotor.setPower(-1);
                backRightMotor.setPower(-1);
            }
            if(gamepad1.dpad_left) {
                frontLeftMotor.setPower(-1);
                backLeftMotor.setPower(1);
                frontRightMotor.setPower(1);
                backRightMotor.setPower(-1);
            }
            if(gamepad1.dpad_right) {
                frontLeftMotor.setPower(1);
                backLeftMotor.setPower(-1);
                frontRightMotor.setPower(-1);
                backRightMotor.setPower(1);
            }

            if(gamepad1.right_bumper) {
                aprilTagRunning = true;

            }
            if(aprilTagRunning) {
                frontLeftMotor.setPower(0.3);
                backLeftMotor.setPower(-0.3);
                frontRightMotor.setPower(-0.3);
                backRightMotor  .setPower(0.3);
                telemetry.addData("April Tag detected: ", tagProcessor.getDetections().size() > 0);
                telemetry.update();
                myAprilTagDetections = tagProcessor.getDetections();

                if (myAprilTagDetections.size() > 0) {
                    for (int i = 0; i < myAprilTagDetections.size(); i++) {
                        AprilTagDetection myAprilTagDetection = myAprilTagDetections.get(i);

                        if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                            myAprilTagIdCode = myAprilTagDetection.id;
                            if(myAprilTagIdCode == 2) {
                                aprilTagRunning = false;
                                frontLeftMotor.setPower(0.2);
                                backLeftMotor.setPower(-0.2);
                                frontRightMotor.setPower(-0.2);
                                backRightMotor  .setPower(0.2);
                                sleep(750);
                            }
                        }
                    }

                }
            }
            telemetry.addData("Motor 1 Left Front",leftFront);
            telemetry.addData("Motor 2 Left Rear", leftRear);
            telemetry.addData("Motor 3 Right Front",rightFront);
            telemetry.addData("Motor 4 Right Rear", rightRear);
            telemetry.update();
        }

    }


}

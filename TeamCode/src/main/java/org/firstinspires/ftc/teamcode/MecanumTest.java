package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.*;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import static android.os.SystemClock.sleep;
import java.util.List;

@TeleOp(name="mecanumtest")
public class MecanumTest extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

    AprilTagProcessor tagProcessor;

    VisionPortal visionPortal;

    private AprilTagDetection tryDetectApriTag(int idCode)
    {
        AprilTagDetection aprilTagDetection = null;
        List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();
        for (int i = 0; i < myAprilTagDetections.size(); i++) {
            AprilTagDetection myAprilTagDetection = myAprilTagDetections.get(i);

            if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                int myAprilTagIdCode = myAprilTagDetection.id;
                if (myAprilTagIdCode == idCode) {
                    aprilTagDetection = myAprilTagDetection;
                    telemetry.addData("y", myAprilTagDetection.ftcPose.y);
                    telemetry.addData("z", myAprilTagDetection.ftcPose.z);
                    telemetry.addData("roll", myAprilTagDetection.ftcPose.roll);
                    telemetry.addData("pitch", myAprilTagDetection.ftcPose.pitch);
                    telemetry.addData("yaw", myAprilTagDetection.ftcPose.yaw);
                    telemetry.update();
                }
            }
        }
        return aprilTagDetection;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        motor4.setDirection(DcMotorSimple.Direction.FORWARD);

        int myAprilTagIdCode = -1;
        int targetAprilTag = 2;
        boolean aprilTagRunning = false;
        // mode 0 : scanning
        // mode 1 : approaching
        int aprilTagMode = 0;
        double desiredDistance = 7;
        boolean aprilTagDetected = false;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        double distance = Double.MAX_VALUE;

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

            telemetry.addData("April Tag detected: ", tagProcessor.getDetections().size() > 0);

            aprilTagDetected = false;
            AprilTagDetection myAprilTagDetection = tryDetectApriTag(targetAprilTag);

            if (myAprilTagDetection != null)
            {
                distance = myAprilTagDetection.ftcPose.y;
                aprilTagDetected = true;
            }

            motor1.setPower(leftFront);
            motor2.setPower(leftRear);
            motor3.setPower(rightFront);
            motor4.setPower(rightRear);

            if(gamepad1.dpad_up) {
                motor1.setPower(1);
                motor2.setPower(1);
                motor3.setPower(1);
                motor4.setPower(1);
            }
            if(gamepad1.dpad_down) {
                motor1.setPower(-1);
                motor2.setPower(-1);
                motor3.setPower(-1);
                motor4.setPower(-1);
            }
            if(gamepad1.dpad_left) {
                motor1.setPower(-1);
                motor2.setPower(1);
                motor3.setPower(1);
                motor4.setPower(-1);
            }
            if(gamepad1.dpad_right) {
                motor1.setPower(1);
                motor2.setPower(-1);
                motor3.setPower(-1);
                motor4.setPower(1);
            }

            if(gamepad1.right_bumper) {
                aprilTagRunning = true;
            }

            if(aprilTagRunning) {
                if (aprilTagDetected && aprilTagMode == 0) {
                    aprilTagMode = 1;
                }
                else if (aprilTagDetected && aprilTagMode == 1){
                    double difference = distance - desiredDistance;

                    // estimating that it takes 170 ms for robot to move 1 inch forward (power 0.2)
                    if (difference > 0.1) {
                        motor1.setPower(0.2);
                        motor2.setPower(0.2);
                        motor3.setPower(0.2);
                        motor4.setPower(0.2);

                        sleep((long) (170 * difference));
                    }
                    else if (difference < -0.1) {

                        motor1.setPower(-0.2);
                        motor2.setPower(-0.2);
                        motor3.setPower(-0.2);
                        motor4.setPower(-0.2);
                        sleep((long) (170 * abs(difference)));
                    }
                    aprilTagRunning = false;
                    aprilTagMode = 0;
                    motor1.setPower(0.2);
                    motor2.setPower(-0.2);
                    motor3.setPower(-0.2);
                    motor4  .setPower(0.2);
                    sleep(750);
                }
                else if (aprilTagDetected == false && aprilTagMode == 0) {
                    motor1.setPower(0.3);
                    motor2.setPower(-0.3);
                    motor3.setPower(-0.3);
                    motor4.setPower(0.3);
                }
            }

            telemetry.addData("Motor 1 Left Front",leftFront);
            telemetry.addData("Motor 2 Left Rear", leftRear);
            telemetry.addData("Motor 3 Right Front",rightFront);
            telemetry.addData("Motor 4 Right Rear", rightRear);
            telemetry.addData("distance",distance);

            telemetry.update();
        }

    }



}

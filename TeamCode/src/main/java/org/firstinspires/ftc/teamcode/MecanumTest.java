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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="mecanumtest")
public class MecanumTest extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

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

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
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



            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                distance = tag.ftcPose.y;
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }

            if (distance < 8) {
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);
            }
            else {
                motor1.setPower(leftFront);
                motor2.setPower(leftRear);
                motor3.setPower(rightFront);
                motor4.setPower(rightRear);
            }

            if (distance < 8 && gamepad1.a && gamepad1.b) {
                motor1.setPower(-leftFront);
                motor2.setPower(-leftRear);
                motor3.setPower(-rightFront);
                motor4.setPower(-rightRear);
            }
            else {
                motor1.setPower(leftFront);
                motor2.setPower(leftRear);
                motor3.setPower(rightFront);
                motor4.setPower(rightRear);
            }

            telemetry.addData("Motor 1 Left Front",leftFront);
            telemetry.addData("Motor 2 Left Rear", leftRear);
            telemetry.addData("Motor 3 Right Front",rightFront);
            telemetry.addData("Motor 4 Right Rear", rightRear);
            telemetry.addData("y",distance);

            telemetry.update();
        }

    }

}

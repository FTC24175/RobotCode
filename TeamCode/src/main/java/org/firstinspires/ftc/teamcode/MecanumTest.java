package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.*;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import static android.os.SystemClock.sleep;
import java.util.List;

@TeleOp(name="mecanumtest")
public class MecanumTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap,telemetry);

        int myAprilTagIdCode = -1;
        int targetAprilTag = 4;
        boolean aprilTagRunning = false;
        // mode 0 : scanning
        // mode 1 : approaching
        int aprilTagMode = 0;
        double desiredDistance = 7;
        int alliance = 1;
        // 0 : blue
        // 1 : red
        boolean aprilTagDetected = false;

        double distance = Double.MAX_VALUE;

        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            //make sure ^^ is negated
            double turn = gamepad1.right_stick_x;

            robot.move(x,y,turn,1);

            telemetry.addData("April Tag detected: ", robot.tagProcessor.getDetections().size());

            aprilTagDetected = false;
            AprilTagDetection myAprilTagDetection = robot.tryDetectApriTag(targetAprilTag);

            if (myAprilTagDetection != null)
            {
                distance = myAprilTagDetection.ftcPose.y;
                aprilTagDetected = true;
            }

            if(gamepad1.dpad_up) {
                robot.move(0,1,0,0.5);
            }
            if(gamepad1.dpad_down) {
                robot.move(0,-1,0,0.5);
            }
            if(gamepad1.dpad_left) {
                robot.move(-1,0,0,0.5);
            }
            if(gamepad1.dpad_right) {
                robot.move(1,0,0,0.5);
            }

            if(gamepad1.right_bumper) {
                aprilTagRunning = true;
            }

            if(aprilTagRunning) {
                if (aprilTagDetected && aprilTagMode == 0) {
                    aprilTagMode = 1;
                }
                else if (aprilTagDetected && aprilTagMode == 1) {
                    double difference = distance - desiredDistance;
                    // estimating that it takes 170 ms for robot to move 1 inch forward (power 0.15)
                    if (difference > 0.1) {
                        robot.move(0, 1, 0, 0.15);
/////////////////////////going up

                        sleep((long) (170 * difference));
                    } else if (difference < -0.1) {
                        robot.move(0, -1, 0, 0.15);
/////////////////////////going down
                        sleep((long) (170 * abs(difference)));
                    }
                    aprilTagRunning = false;
                    aprilTagMode = 0;

                    if (alliance == 0) {
                        robot.move(1, 0, 0, 0.2);
                        sleep(750);
                    } else if (alliance == 1) {
                        robot.move(1, 0, 0, 0.2);
                        sleep(300);
                    }
                }
                else if (aprilTagDetected == false && aprilTagMode == 0) {
                    if (alliance == 0)
                    {
                        robot.move(1,0,0,0.3);
                    }
                    else if (alliance == 1)
                    {
                        robot.move(-1,0,0,0.3);
                    }
                }
            }

            telemetry.addData("distance",distance);
            telemetry.update();
        }

    }



}

package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Redclose")
public class Redclose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap,telemetry);
        boolean aprilTagDetected = false;
        int aprilTagMode = 0;
        int targetAprilTag = 4;
        int alliance = 1;
        double desiredDistance = 7;
        double distance = Double.MAX_VALUE;
        boolean aprilTagRunning = true;

        while (opModeInInit())
        {

        }

        while (aprilTagRunning) {

            aprilTagDetected = false;
            AprilTagDetection myAprilTagDetection = robot.tryDetectApriTag(targetAprilTag);

            if (myAprilTagDetection != null)
            {
                distance = myAprilTagDetection.ftcPose.y;
                aprilTagDetected = true;
            }

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
                aprilTagMode = 0;

                if (alliance == 0) {
                    robot.move(1, 0, 0, 0.2);
                    sleep(750);
                } else if (alliance == 1) {
                    robot.move(1, 0, 0, 0.2);
                    sleep(300);
                }

                aprilTagRunning = false;
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
    }
}

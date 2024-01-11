package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "RedClose")
public class RedClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot(this);
        robot.initialize();
        boolean debugMode = true;
        boolean aprilTagDetected = false;
        int aprilTagMode = 0;
        int targetAprilTag = 4;
        int alliance = 1;
        int red = robot.getColorSensorRed();
        int i = 0;
        double desiredDistance = 7;
        double distance = Double.MAX_VALUE;
        boolean aprilTagRunning = true;
        boolean checkForRed = true;

        while (opModeInInit())
        {
            telemetry.addData("Left Distance Sensor", String.format("%.01f cm", robot.distanceSensorL.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right Distance Sensor", String.format("%.01f cm", robot.distanceSensorR.getDistance(DistanceUnit.CM)));
            telemetry.addData("Left Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawL.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawR.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
        //move robot to red lines

        // object detection
        for (i=0; i<50; i++) {
            sleep(100);
            if (robot.distanceSensorL.getDistance(DistanceUnit.CM) < 10) {
                break;
            }
            else if (robot.distanceSensorR.getDistance(DistanceUnit.CM) < 10) {
                break;
            }
            else if (robot.distanceSensorClawR.getDistance(DistanceUnit.CM) < 3) {
                break;
            }
            if (debugMode == true) {
                telemetry.addData("Left Distance Sensor", String.format("%.01f cm", robot.distanceSensorL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Distance Sensor", String.format("%.01f cm", robot.distanceSensorR.getDistance(DistanceUnit.CM)));
                telemetry.addData("Left Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawR.getDistance(DistanceUnit.CM)));
            }
            telemetry.update();
        }
        // once detected, stop the robot
        robot.move(0,0,0,0);
        sleep(1000);

        //move robot back
        robot.move(0,-1,0,0.4);
        sleep(900);
        robot.move(0,0,0,0);
        sleep(200);
        //turn to face backdrop

        robot.move(0,0,1,0.4);
        sleep(1150);
        robot.move(0,0,0,0);
        //Move robot forward until it senses red

        robot.move(0,1,0,0.4);
        sleep(1000);
        robot.move(0,1,0,0.2);
        while (checkForRed) {
            red = robot.getColorSensorRed();
            if(red >= robot.getDefaultRed()+MecanumRobot.red_diff) {
                robot.move(0,0,0,0);
                checkForRed = false;
            }
            telemetry.addData("Red: ", red);
            telemetry.addData("Red threshold: ", MecanumRobot.red_threshold);
            telemetry.update();
            sleep(10);
        }
        //start scanning for april tag

        // april tag start
        if (alliance == 0)
        {
            robot.move(1,0,0,0.2);
        }
        else if (alliance == 1)
        {
            robot.move(-1,0,0,0.2);
        }
        while (aprilTagRunning && opModeIsActive()) {

            aprilTagDetected = false;
            AprilTagDetection myAprilTagDetection = robot.tryDetectApriTag(targetAprilTag);
            telemetry.addData("April Tag detected: ", robot.getDetectionSize());

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
                    robot.move(1, 0, 0, 0.3);
                    sleep(750);
                } else if (alliance == 1) {
                    robot.move(1, 0, 0, 0.3);
                    sleep(300);
                }

                aprilTagRunning = false;
            }


            telemetry.update();
            sleep(10);
        }
        robot.move(0,0,-1,0.4);
        sleep(2400);
    }
}

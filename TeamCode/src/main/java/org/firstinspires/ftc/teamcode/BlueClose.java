package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "BlueClose")
public class BlueClose extends LinearOpMode {

    MecanumRobot robot = new MecanumRobot(this);
    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize();

        boolean aprilTagDetected = false;
        int aprilTagMode = 0;
        int targetAprilTag = 2;
        int alliance = 0;
        int blue = robot.getColorSensorBlue();
        double desiredDistance = 7;
        double distance = Double.MAX_VALUE;
        boolean aprilTagRunning = true;
        boolean checkForBlue = true;

        int teamPropLocation = 0;

        while (opModeInInit())
        {
            telemetry.update();
        }
        //move robot to red lines

        robot.move(0,1,0,0.4);
        sleep(1000);
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
        sleep(1000); // move forward using power 0.4 for 1 second
        robot.move(0,1,0,0.2);
        while (checkForBlue) { // move forward using power 0.2 until blue line is detected
            blue = robot.getColorSensorBlue();
            // default blue value in the gray (178)
            // blue value in the blue line (245...increased by 67)
            if(blue >= robot.getDefaultBlue() + 500) { // detects blue line
                robot.move(0,0,0,0); // brakes
                checkForBlue = false; // will break the while loop
            }
            telemetry.addData("Blue: ", blue);
            telemetry.addData("initial blue: ", robot.getDefaultBlue());
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

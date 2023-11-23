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
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="mecanumtest")
public class MecanumTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.initialize(hardwareMap,telemetry);

        int myAprilTagIdCode = -1;
        int targetAprilTag = 2;
        boolean aprilTagRunning = false;
        boolean checkForRed = false;
        boolean checkForBlue = false;
        // mode 0 : scanning
        // mode 1 : approaching
        int aprilTagMode = 0;
        double desiredDistance = 7;
        boolean aprilTagDetected = false;

        double distance = Double.MAX_VALUE;

        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            //make sure ^^ is negated
            double turn = gamepad1.right_stick_x;

            int red = robot.colorSensor.red();   // Red channel value
            int green = robot.colorSensor.green(); // Green channel value
            int blue = robot.colorSensor.blue();  // Blue channel value
            int alpha = robot.colorSensor.alpha(); // Total luminosity
            int argb = robot.colorSensor.argb();  // Combined color value

            robot.move(x,y,turn,1);

            telemetry.addData("April Tag detected: ", robot.tagProcessor.getDetections().size() > 0);

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
            if(gamepad1.x) {
                checkForRed = true;
            }
            if(gamepad1.y) {
                checkForBlue = true;
            }

            if(aprilTagRunning) {
                if (aprilTagDetected && aprilTagMode == 0) {
                    aprilTagMode = 1;
                }
                else if (aprilTagDetected && aprilTagMode == 1){
                    double difference = distance - desiredDistance;
                    // estimating that it takes 170 ms for robot to move 1 inch forward (power 0.2)
                    if (difference > 0.1) {
                        robot.move(0,-1,0,0.2);
/////////////////////////going up

                        sleep((long) (170 * difference));
                    }
                    else if (difference < -0.1) {
                        robot.move(0,1,0,0.2);

/////////////////////////going down

                        sleep((long) (170 * abs(difference)));
                    }
                    aprilTagRunning = false;
                    aprilTagMode = 0;
                    robot.move(1,0,0,0.2);
                    sleep(750);
                }
                else if (aprilTagDetected == false && aprilTagMode == 0) {
                    robot.move(1,0,0,0.3);
                }
            }
            if(checkForRed) {
                if(red < robot.default_red + 500) {
                    robot.move(0,1,0,0.3);
                }
                else  {
                    checkForRed = false;
                }
            }

            if(checkForBlue) {
                if (blue < robot.default_blue + 500) {
                    robot.move(0, 1, 0, 0.3);
                } else {
                    checkForBlue = false;
                }
            }
            telemetry.addData("Red: ", red);
            telemetry.addData("Blue: ", blue);
            telemetry.addData("Checking for Red: ", checkForRed);
            telemetry.addData("Checking for Blue: ", checkForBlue);
            telemetry.addData("distance",distance);
            telemetry.update();
        }

    }



}

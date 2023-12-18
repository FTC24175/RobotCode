package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.*;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import static android.os.SystemClock.sleep;
import java.util.List;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="MecanumTest")
public class MecanumTest extends LinearOpMode {

    MecanumRobot robot = new MecanumRobot(this);
    int clicks = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // manual mode
        double leftPower = 0;
        double rightPower = 0;
        double slidePower = 0;
        double leftPosition = 0;
        double rightPosition = 0;
        double wristPosition = 0;
        robot.setServoPositionWrist(wristPosition);
        robot.setServoPositionLeftHand(leftPosition);
        robot.setServoPositionRightHand(rightPosition);

        /* // Manual mode By Bo
        int liftMax = 550;
        int wristMax = 550;
        double leftArmPosition = robot.getLeftArmPosition();
        double wristPosition = robot.getWristPosition();
        double clawPosition = Constants.clawOpen;
        */

        /* Auto mode
        * April Tag
         */

        // int myAprilTaIdCode = -1; // not used
        // int targetAprilTag = 2; // not used

        // mode 0 : scanning
        // mode 1 : approaching
        int aprilTagMode = 0;
        boolean aprilTagRunning = Constants.aprilTagRunning;
        boolean aprilTagDetected = false;

        boolean checkForRed = false;
        boolean checkForBlue = false;
        // mode 0 : scanning
        // mode 1 : approaching

        double desiredDistance = 7;
        int alliance = 1;

        // 0 : blue
        // 1 : red
        int red = 0;
        int blue = 0;

        double distance = Double.MAX_VALUE;

        waitForStart();

        while(opModeIsActive()) {

            // Mecanum Drivetrain By Kush & Derek 11/18-11/22

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            //make sure ^^ is negated
            double turn = gamepad1.right_stick_x;

            robot.move(x, y, turn, 1);

            if (gamepad1.dpad_up) {
                robot.move(0, 1, 0, 0.5);
            }
            if (gamepad1.dpad_down) {
                robot.move(0, -1, 0, 0.5);
            }
            if (gamepad1.dpad_left) {
                robot.move(-1, 0, 0, 0.5);
            }
            if (gamepad1.dpad_right) {
                robot.move(1, 0, 0, 0.5);
            }

            // Manual mode By Kush on 11/29/2023

            // Arm movement

            if (gamepad1.x) {

                //robot.motor1ex.setPower(0.5);
                //robot.motor2ex.setPower(0.5);

                leftPower = 0.5;
                rightPower = 0.5;
                robot.setMotorPowerArm(leftPower);
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();


            } else if (gamepad1.y) {

                //robot.motor1ex.setPower(-0.5);
                //robot.motor2ex.setPower(-0.5);

                leftPower = -0.5;
                rightPower = -0.5;
                robot.setMotorPowerArm(leftPower);
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();

            } else { // when nothing is pressed, brake the arm motors

                //robot.motor1ex.setPower(0);
                //robot.motor2ex.setPower(0);

                leftPower = 0;
                rightPower = 0;
                robot.setMotorPowerArm(leftPower);
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }

            telemetry.addData("Left Arm Position",robot.getMotorPositionLeftArm());
            telemetry.update();


            // Slide movement
            //Need to set max

            if(gamepad2.right_stick_y > 0){
                //robot.setMotorTargetSlide(300);
                robot.setMotorPowerSlide(gamepad2.right_stick_y);

            }
            else if (gamepad2.right_stick_y < 0){
                //robot.setMotorTargetSlide(300);
                robot.setMotorPowerSlide(gamepad2.right_stick_y);
            }
            else{
                //robot.setMotorTargetSlide(0);
                robot.setMotorPowerSlide(0);
            }

            telemetry.addData("Slide Position",robot.getMotorPositionSlide());
            telemetry.update();


            // Hand movement

            if (gamepad2.left_trigger > 0.3) {

                if (robot.getServoPositionLeftHand() == 1) {

                    //robot.servo1.setPosition(0);
                    //robot.servo2.setPosition(0);
                    leftPosition = 0;
                    robot.setServoPositionLeftHand(leftPosition);
                    telemetry.addData("Servos", "left (%.2f), right (%.2f)", leftPosition, rightPosition);
                    telemetry.update();

                } else {

                    //robot.servo1.setPosition(1);
                    //robot.servo2.setPosition(1);
                    leftPosition = 1;
                    robot.setServoPositionLeftHand(leftPosition);
                    telemetry.addData("Servos", "left (%.2f), right (%.2f)", leftPosition, rightPosition);
                    telemetry.update();
                }

                sleep(300); // wait for 0.3 second

            }

            if (gamepad2.right_trigger > 0.3) {

                if (robot.getServoPositionRightHand() == 1) {

                    //robot.servo1.setPosition(0);
                    //robot.servo2.setPosition(0);
                    rightPosition = 0;
                    robot.setServoPositionRightHand(rightPosition);
                    telemetry.addData("Servos", "left (%.2f), right (%.2f)", leftPosition, rightPosition);
                    telemetry.update();

                } else {

                    //robot.servo1.setPosition(1);
                    //robot.servo2.setPosition(1);
                    rightPosition = 1;
                    robot.setServoPositionRightHand(rightPosition);
                    telemetry.addData("Servos", "left (%.2f), right (%.2f)", leftPosition, rightPosition);
                    telemetry.update();
                }

                sleep(300); // wait for 0.3 second

            }

            // Wrist movement

            //*In progress - Only has 2 positions: 0 and 1*
            //We want to make it go up in increments so it's easier for drivers to use the wrist

                if (gamepad2.left_stick_y > 0) {
                    if (wristPosition < 1) {
                        wristPosition += 0.1;
                    }
                }
                else if(gamepad2.left_stick_y < 0){
                    if(wristPosition > 0){
                        wristPosition-=0.1;
                    }
                }

                robot.setServoPositionWrist(wristPosition);






            /*
            // Intake wheels by Kush

            if (gamepad1.a) {
                robot.motor3ex.setPower(0.3);
            } else if (gamepad1.b) {
                robot.motor3ex.setPower(-0.3);
            } else {
                robot.motor3ex.setPower(0);
            }
            */

            /*
            * Manual mode by Bo on 12/1
             */
            /*
            //move arm up and down when x & y buttons are pressed

            if(gamepad1.y)
            {
                if(leftArmPosition >=liftMax) {
                    robot.motor1ex.setPower(0);
                    robot.motor2ex.setPower(0);
                }
                else {
                    robot.motor1ex.setPower(0.6);
                    robot.motor2ex.setPower(0.6);
                    leftArmPosition = robot.motor1ex.getCurrentPosition();
                }
            }
            else if(gamepad1.x) {
                robot.motor1ex.setPower(-0.6);
                robot.motor2ex.setPower(-0.6);
                leftArmPosition = robot.motor1ex.getCurrentPosition();
            }
            else
            {
                robot.motor1ex.setPower(0);
                robot.motor2ex.setPower(0);
            }

            //move wrist up and down when a & b buttons are pressed

            if (gamepad1.b)
            {
                if(wristPosition < Constants.wristUp) {
                    wristPosition += 0.02;
                    robot.servo1.setPosition(wristPosition);

                }
                sleep(10);
            }
            if (gamepad1.a)
            {
                if(wristPosition > Constants.wristDown) {
                    wristPosition -= 0.02;
                    robot.servo1.setPosition(wristPosition);

                }
                sleep(10);
            }

            //open and close one claw

            if (gamepad1.left_bumper) {
                clicks += 1;
                if (clicks == 2){
                    if (clawPosition == Constants.clawOpen) {
                        clawPosition = Constants.clawClose;
                    } else if (clawPosition == Constants.clawClose) {
                        clawPosition = Constants.clawOpen;
                    }
                    robot.servo3.setPosition(clawPosition);
                    sleep(200);
                    clicks = 0;
                }
            }

            //open and close the other claw

            if (gamepad1.right_bumper) {
                clicks += 1;
                if (clicks == 2){
                    if (clawPosition == Constants.clawOpen) {
                        clawPosition = Constants.clawClose;
                    } else if (clawPosition == Constants.clawClose) {
                        clawPosition = Constants.clawOpen;
                    }
                    robot.servo2.setPosition(clawPosition);
                    sleep(200);
                    clicks = 0;
                }
            }

            // move the wheel out
            if (gamepad1.left_trigger > 0.3){
                robot.motor3ex.setPower(0.5);
            }

            // move the wheel in
            if (gamepad1.right_trigger > 0.3){
                robot.motor3ex.setPower(-0.5);
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                if (clawPosition == Constants.clawOpen) {
                    clawPosition = Constants.clawClose;
                } else if (clawPosition == Constants.clawClose) {
                    clawPosition = Constants.clawOpen;
                }
                robot.servo3.setPosition(clawPosition);
                robot.servo2.setPosition(clawPosition);
                sleep(200);
                clicks = 0;

            }


            if (gamepad1.left_trigger > 0.3 && gamepad1.right_trigger > 0.3) {
                robot.servo4.setPosition(0.5);
            }

            //if (gamepad1.start) {
            //    robot.servo4
            //}
        */
            /* Auto mode
            * April Tag
             */
            /*
            // By Kush
            if (gamepad1.right_bumper) {
                aprilTagRunning = true;
            }
            */
            if (aprilTagRunning) {
                if (gamepad1.x) {
                    checkForRed = true;
                }
                if (gamepad1.y) {
                    checkForBlue = true;
                }
                if (aprilTagDetected && aprilTagMode == 0) {
                    aprilTagMode = 1;
                } else if (aprilTagDetected && aprilTagMode == 1) {
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
                } else if (aprilTagDetected == false && aprilTagMode == 0) {
                    if (alliance == 0) {
                        robot.move(1, 0, 0, 0.3);
                    } else if (alliance == 1) {
                        robot.move(-1, 0, 0, 0.3);
                    }
                }
                if (checkForRed) {
                    if (red < robot.getDefaultRed() + 500) {
                        robot.move(0, 1, 0, 0.3);
                    } else {
                        checkForRed = false;
                    }
                }

                if (checkForBlue) {
                    if (blue < robot.getDefaultBlue() + 500) {
                        robot.move(0, 1, 0, 0.3);
                    } else {
                        checkForBlue = false;
                    }
                }

                telemetry.addData("Red: ", red);
                telemetry.addData("Blue: ", blue);
                telemetry.addData("Checking for Red: ", checkForRed);
                telemetry.addData("Checking for Blue: ", checkForBlue);
                telemetry.addData("distance", distance);
                telemetry.update();
            }
        }
    }
}





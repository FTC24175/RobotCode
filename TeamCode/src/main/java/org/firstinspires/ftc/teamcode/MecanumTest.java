package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.*;

import android.transition.Slide;
import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import static android.os.SystemClock.sleep;
import java.util.List;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="MecanumTest")
public class MecanumTest extends LinearOpMode {

    MecanumRobot robot = new MecanumRobot(this);
    boolean debugMode = true;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize();
        telemetry.addData("Status", "Initialized");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // manual mode
        double leftPower = 0;
        double rightPower = 0;
        double slidePower = 0;
        double leftPosition = 0;
        double rightPosition = 1;
        double wristPosition = 0;
        double launcherPosition = 0;
        double slideMax = 10119;
        double slideMin = 0;
        double armMax = 16051;
        double armMin = 0;
        boolean armDown = false;

        robot.setServoPositionWrist(wristPosition);
        robot.setServoPositionLeftHand(leftPosition);
        robot.setServoPositionRightHand(rightPosition);
        robot.setServoPositionLauncher(launcherPosition);
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

        //Calibrate arm & slide position - buggy
        // (Rachel: Now the new encoder remembers the position from last time,
        // So there no need to calibrate)
        /*
        if (opModeIsActive()) {
            if (robot.touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
                robot.setMotorPowerArm(0);
                //robot.setMotorPowerSlide(0);
                robot.encoderReset();
                break;
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
                robot.setMotorPowerArm(-0.2);
                //robot.setMotorPowerSlide(-0.2);
            }
            telemetry.update();
        }
        */
        int leftArmPosition;
        int slidePosition;
        int servoPosition;

        while(opModeIsActive()) {

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

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

            // Touch sensor
            // Cannot use port 0 on driver station
            if (robot.touchSensor.isPressed()) {
                if (debugMode == true)
                    telemetry.addData("Touch Sensor", "Is Pressed");
                armDown = true;

            } else {
                if (debugMode == true)
                    telemetry.addData("Touch Sensor", "Is Not Pressed");
                armDown = false;
            }
            /*
            // Arm movement by gamepad 1
            // up
            leftArmPosition = robot.getMotorPositionLeftArm();
            telemetry.addData("Left Arm Position", leftArmPosition);
            if (gamepad1.y) {
                if (leftArmPosition < armMax) {
                    leftPower = 0.4;
                    rightPower = 0.4;
                    robot.setMotorPowerArm(leftPower);
                    telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                }
            } //down
            else if (gamepad1.x) {
                if (!armDown) {
                    leftPower = -0.2;
                    rightPower = -0.2;
                    robot.setMotorPowerArm(leftPower);
                    telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                }
            }
            */
            // Arm movement gamepad 2
            leftArmPosition = robot.getMotorPositionLeftArm();
            if (gamepad2.right_stick_y > 0) { // Move arm down
                if (debugMode == true)
                    telemetry.addData("Right Joystick:", gamepad2.right_stick_y);
                if (leftArmPosition <= armMin) {
                    robot.setMotorPowerArm(0); // brake
                    if (debugMode == true)
                        telemetry.addData("Lower arm. Brake arm motor", 0);
                } else {
                    leftPower = -gamepad2.right_stick_y/3;
                    robot.setMotorPowerArm(leftPower);
                    if (debugMode == true)
                        telemetry.addData("Lower arm. Set Arm Power to ", -leftPower);
                }
            }
            else if (gamepad2.right_stick_y < 0) { // Move arm up
                if (debugMode == true)
                    telemetry.addData("Right Joystick:", gamepad2.right_stick_y);
                if (leftArmPosition >= armMax) {
                    robot.setMotorPowerArm(0); // brake
                    if (debugMode == true)
                        telemetry.addData("Raise arm. Brake arm motor", 0);
                } else {
                    leftPower = -gamepad2.right_stick_y/3;
                    robot.setMotorPowerArm(leftPower);
                    if (debugMode == true)
                        telemetry.addData("Raise arm. Set Arm Power to ", -leftPower);
                }

            }
            else { // when nothing is pressed, brake the arm motors
                leftPower = 0;
                rightPower = 0;
                robot.setMotorPowerArm(-leftPower);
                if (debugMode == true)
                    telemetry.addData("Motors", "left (%.2f), right (%.2f)", -leftPower, -rightPower);
            }
            if (debugMode == true)
                telemetry.addData("Arm" + "" + " New Position", leftArmPosition);
            /*
            * Slide movement
            * It's against intuition that the joystick gives a negative value when it is pushed up
             */
            slidePosition = robot.getMotorPositionSlide();
            if (gamepad2.left_stick_y > 0) { // joystick down & retract
                if (slidePosition <= slideMin) {
                    robot.setMotorPowerSlide(0); // brake
                    if (debugMode == true)
                        telemetry.addData("Extend arm. Brake slide motor", 0);
                } else {
                    slidePower = -gamepad2.left_stick_y/2;
                    robot.setMotorPowerSlide(slidePower);
                    if (debugMode == true)
                        telemetry.addData("Retract arm. Set Slide Power to ", -slidePower);
                }
            } else if (gamepad2.left_stick_y < 0) { // joystick up & extend
                if (slidePosition >= slideMax) {
                    robot.setMotorPowerSlide(0); // brake
                    if (debugMode == true)
                        telemetry.addData("Extend arm. Brake slide motor", 0);
                } else {
                    slidePower = -gamepad2.left_stick_y/2;
                    robot.setMotorPowerSlide(slidePower);
                    if (debugMode == true)
                        telemetry.addData("Extend arm. Set Slide Power to ", -slidePower);
                }

            } else { // brake
                robot.setMotorPowerSlide(0);
            }
            if (debugMode == true)
                telemetry.addData("Slide New Position", slidePosition);


            // Hand movement

//////////////// Make it harder to lose pixels
//////////////// Only pressing both triggers will open the claws

//////////////// Solve another problem: Two pixels need to fall next to each other on the backdrop
//////////////// How to release pixels so the problem won't occur?
            leftPosition = robot.getServoPositionLeftHand();
            rightPosition = robot.getServoPositionRightHand();
            if (gamepad2.left_trigger > 0.3) {
                if ((leftPosition == 0) && (gamepad2.a)) { //close
                    leftPosition = 0.5; //open
                    robot.setServoPositionLeftHand(leftPosition);
                    sleep(100);
                }
                else {
                    leftPosition = 0; //close
                    robot.setServoPositionLeftHand(leftPosition);
                    sleep(100);
                }
            }
            if (gamepad2.right_trigger > 0.3) {
                if ((rightPosition == 1) && (gamepad2.a)) { //close
                    rightPosition = 0.5; //open
                    robot.setServoPositionRightHand(rightPosition);
                    sleep(100);
                } else {
                    rightPosition = 1; //close
                    robot.setServoPositionRightHand(rightPosition);
                    sleep(100);
                }
            }

            if (gamepad1.left_trigger > 0.3) {
                if (leftPosition == 0) { //close
                    leftPosition = 0.5; //open
                    robot.setServoPositionLeftHand(leftPosition);
                    sleep(100);
                } else {
                    leftPosition = 0; //close
                    robot.setServoPositionLeftHand(leftPosition);
                    sleep(100);
                }
            }
            if (gamepad1.right_trigger > 0.3) {
                if (rightPosition == 1) {
                    rightPosition = 0.5; //open
                    robot.setServoPositionRightHand(rightPosition);
                    sleep(100);
                }
                else {
                    rightPosition = 1; //close
                    robot.setServoPositionRightHand(rightPosition);
                    sleep(100);

                }
            }
            if (debugMode == true)
                telemetry.addData("Claw Servos", "left (%.2f), right (%.2f)", leftPosition, rightPosition);

            // Wrist movement
            wristPosition = robot.getServoPositionWrist();
            if ((gamepad1.a) || (gamepad2.dpad_down)) { //wrist down
                if (wristPosition < 1) {
                    wristPosition += 0.05;
                    robot.setServoPositionWrist(wristPosition);
                }
            } else if ((gamepad1.b) || (gamepad2.dpad_up)) { //wrist up
                if (wristPosition > 0) {
                    wristPosition -= 0.05;
                    robot.setServoPositionWrist(wristPosition);
                }
            }
            if (debugMode == true)
                telemetry.addData("Wrist Servo", "%.2f", wristPosition);

            //Launcher
            if ((gamepad1.back) || (gamepad2.back)) {
                launcherPosition = 1;
                robot.setServoPositionLauncher(launcherPosition);
            }

            //Distance Sensor
            if (debugMode == true) {
                telemetry.addData("Left Distance Sensor", String.format("%.01f cm", robot.distanceSensorL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Distance Sensor", String.format("%.01f cm", robot.distanceSensorR.getDistance(DistanceUnit.CM)));
                telemetry.addData("Left Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawL.getDistance(DistanceUnit.CM)));
                telemetry.addData("Right Claw Distance Sensor", String.format("%.01f cm", robot.distanceSensorClawR.getDistance(DistanceUnit.CM)));
            }
            telemetry.update();

            //Automatic Arm Down
            if (gamepad2.x) {
                if (armDown != true)
                    robot.AutoArmDown();
            }
            if (gamepad2.y) {

                robot.AutoArmUp();
            }

            if ((gamepad2.right_bumper) && (gamepad2.left_bumper)) {
                robot.AutoLinePark();
            }

            if (gamepad1.x)
                robot.AutoWristDown();
            if (gamepad1.y)
                robot.AutoWristUp();


///////////////////////////////Automatic Arm Up

///////////////////////////////Automatic Pixel Pick-up

///////////////////////////////Automatic Park at Line - Moves Forward

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
                } else if (!aprilTagDetected && aprilTagMode == 0) {
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





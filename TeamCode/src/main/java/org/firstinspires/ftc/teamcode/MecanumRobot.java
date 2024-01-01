package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.deprecated.teamTeleOpCode.clawPosition;
//import static org.firstinspires.ftc.teamcode.deprecated.teamTeleOpCode.wristPosition;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

// Autonomous mode
//import com.google.blocks.ftcrobotcontroller.runtime.AprilTagAccess;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class MecanumRobot {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor motorHDLeftFront = null;
    private DcMotor motorHDLeftRear = null;
    private DcMotor motorHDRightFront = null;
    private DcMotor motorHDRightRear = null;

    private DcMotor motorLeftArm = null;
    private DcMotor motorRightArm = null;
    private DcMotor motorSlides = null;
    private Servo servoWrist = null;
    private Servo servoLeftHand = null;
    private Servo servoRightHand = null;
    private Servo servoLauncher = null;

    public TouchSensor touchSensor = null;
    public DistanceSensor distanceSensorL = null;
    public DistanceSensor distanceSensorR = null;
    public DistanceSensor distanceSensorClawL = null;
    public DistanceSensor distanceSensorClawR = null;

    // Auto mode
    private ColorSensor colorSensor, colorSensor2;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    private IMU imu;
    private int default_red;
    private int default_blue;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public MecanumRobot (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initialize()
    {
        // Mecanum Drivertrain Motors

        motorHDLeftFront = myOpMode.hardwareMap.get(DcMotor.class, "HDMotor2");
        motorHDLeftRear = myOpMode.hardwareMap.get(DcMotor.class, "HDMotor3");
        motorHDRightFront = myOpMode.hardwareMap.get(DcMotor.class, "HDMotor0");
        motorHDRightRear = myOpMode.hardwareMap.get(DcMotor.class, "HDMotor1");

        // RUN_WITHOUT_ENCODER mode - only set direction & power

        motorHDLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHDLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHDRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHDRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorHDLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorHDLeftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHDRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHDRightRear.setDirection(DcMotorSimple.Direction.FORWARD);


        // Top Motors

        motorLeftArm = myOpMode.hardwareMap.get(DcMotor.class, "HDMotorArmL");
        motorRightArm = myOpMode.hardwareMap.get(DcMotor.class, "HDMotorArmR");
        motorSlides = myOpMode.hardwareMap.get(DcMotor.class, "CoreMotorSlide");

        // Reset zero position
        // sometimes randomly reverses directions when switched to STOP_AND_RESET_ENCODER mode (reverse -> forward and vice versa)
        motorSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // RUN_WITHOUT_ENCODER mode - only set direction & power

        // Must change to the WITHOUT ENCODER mode; otherwise, the core motor won't move when set power
        motorLeftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);;
        motorSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Try both directions and choose the one that results in positive encoder ticks

        motorLeftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        myOpMode.telemetry.addData("direction of left arm motor","forward");
        motorRightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        myOpMode.telemetry.addData("direction of slide motor","reverse");

        // Servos

        servoWrist = myOpMode.hardwareMap.get(Servo.class, "ServoWrist");
        servoLeftHand = myOpMode.hardwareMap.get(Servo.class, "ServoClawL");
        servoRightHand = myOpMode.hardwareMap.get(Servo.class, "ServoClawR");
        servoLauncher = myOpMode.hardwareMap.get(Servo.class, "ServoLauncher");
        touchSensor = myOpMode.hardwareMap.get(TouchSensor.class, "touchSensor");
        distanceSensorL = myOpMode.hardwareMap.get(DistanceSensor.class, "distanceSensorL");
        distanceSensorR = myOpMode.hardwareMap.get(DistanceSensor.class, "distanceSensorR");
        distanceSensorClawL = myOpMode.hardwareMap.get(DistanceSensor.class, "distanceSensorClawL");
        distanceSensorClawR = myOpMode.hardwareMap.get(DistanceSensor.class, "distanceSensorClawR");
        /*
        * Auto mode initialization
        */

        /*
        //By Bo
        motor0core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */


        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensorR");
        colorSensor.enableLed(true);
        colorSensor2 = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensorL");
        colorSensor2.enableLed(true);

        default_red = colorSensor.red();
        default_blue = colorSensor.blue();

        /* local variables
        int myAprilTaIdCode = -1;
        int targetAprilTag = 2;
        boolean aprilTagRunning = false;
        // mode 0 : scanning
        // mode 1 : approaching
        int aprilTagMode = 0;
        double desiredDistance = 7;
        boolean aprilTagDetected = false;
        double distance = Double.MAX_VALUE;
        */
    }

    /*
    public void encoderReset() {
        motorSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // has to reset right arm, too
        // then switch back to RUN_WITHOUT_ENCODER mode
    }
    */

    /**
     * Mecanum Drivetrain
     * Calculates the left/right front/rear motor powers required to achieve the requested
     * robot motions: ...
     * Then sends these power levels to the motors.
     *
     * @param x
     * @param y
     * @param turn
     * @param powerScale
     */
    public void move(double x, double y, double turn, double powerScale) {
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
        motorHDLeftFront.setPower(leftFront * powerScale);
        motorHDLeftRear.setPower(leftRear * powerScale);
        motorHDRightFront.setPower(rightFront * powerScale);
        motorHDRightRear.setPower(rightRear * powerScale);

        myOpMode.telemetry.addData("Motor 0 Left Front",leftFront * powerScale);
        myOpMode.telemetry.addData("Motor 1 Left Rear", leftRear * powerScale);
        myOpMode.telemetry.addData("Motor 2 Right Front",rightFront * powerScale);
        myOpMode.telemetry.addData("Motor 3 Right Rear", rightRear * powerScale);
    }

    /**
     * Arm movement
     * Set both motor powers
     * Then sends these power levels to the motors.
     *
     * @param powerScale
     */

    public void setMotorPowerArm(double powerScale) {
        motorLeftArm.setPower(powerScale);
        motorRightArm.setPower(powerScale);

    }

    public int getMotorPositionLeftArm(){
        return motorLeftArm.getCurrentPosition();
    }
    public int getMotorPositionSlide(){
        return motorSlides.getCurrentPosition();
    }
    public DcMotorSimple.Direction getMotorDirectionSlide() { return motorSlides.getDirection(); }
    public void setMotorTargetSlide(int targetPosition) {
        motorSlides.setTargetPosition(targetPosition);
        motorSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setMotorPowerSlide(double powerScale) {
        motorSlides.setPower(powerScale);
    }

    public double getServoPositionLeftHand() {
        return servoLeftHand.getPosition();
    }

    public double getServoPositionRightHand() {

        return servoRightHand.getPosition();
    }

    public void setServoPositionLeftHand(double position) {

        servoLeftHand.setPosition(position);

    }

    public void setServoPositionRightHand(double position) {

        servoRightHand.setPosition(position);

    }

    public double getServoPositionWrist() {
        return servoWrist.getPosition();
    }
    public void setServoPositionWrist(double position) {
        servoWrist.setPosition(position);
    }
    public void setServoPositionLauncher(double position) {
        servoLauncher.setPosition(position);
    }

    public void AutoArmDown() {

        // Wrist up to position 0
        servoWrist.setPosition(0);

        // Slide retracts to position 0 -- BUGGY
        motorSlides.setTargetPosition(0);
        motorSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myOpMode.sleep(1000);

        // Reset the slide's encoder and direction as the initial settings
        // so it won't affect another part of the program
        motorSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm down to position 0
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (touchSensor.isPressed() != true && runtime.seconds() < 1.5) {
            setMotorPowerArm(0.3);
        }
    }

    //////////////////////////// Automatic Arm Up
    public void AutoArmUp() {

        /* Raises the left arm to 3628 ticks
         *
         * The right arm motor needs to rotate at the reverse direction, too
         * The absolute positions of the two arms should never be the same
         * but the right arm supposedly travels for same number of ticks as the left arm
         * */

        /*
        // Sample code from the REV Duo Control System Document
        // Encoder Navigation - OnBot Java

        leftmotor.setTargetPosition(leftTarget);
        rightmotor.setTargetPosition(rightTarget);
        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftmotor.setVelocity(TPS);
        rightmotor.setVelocity(TPS);

        // If any motor quits being busy, the other motor stops at the same time
        while (opModeIsActive() && (leftmotor.isBusy() && rightmotor.isBusy())) {
            telemetry.addData("left", leftmotor.getCurrentPosition());
            telemetry.addData("right", rightmotor.getCurrentPosition());
            telemetry.update();
        }
        */

        // Refer to the initialization function to reset encoder mode and the direction for the arm motors

        // Extends the slide to slideMax ticks

        // Refer to the AutoArmDown function to reset encoder mode and the direction for the slide motor

        // Puts down the wrist to position 0.6

    }
    /////////////////////////// Automatic Pixel Pick-up
    // Happens at human player
    public void AutoPickUp() {
        // Opens claws
        // Puts the wrist down
        // Closes claws
        // Puts the wrist up
    }

    // Happens in front of the back board and the human player
    public void AutoParkAtLineForwardRightTurn() {
        // While under a timer
        // Moves forward until a color sensor detects blue or red
        // Rotates right until another color sensor detects blue or red
    }

    public void AutoParkAtLineMarkForwardLeftTurn() {
        // While under a timer
        // Moves forward until a color sensor detects blue or red
        // Rotates left until another color sensor detects blue or red
    }

    public void intializeAprilTag()
    {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public AprilTagDetection tryDetectApriTag(int idCode)
    {
        AprilTagDetection aprilTagDetection = null;
        List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();
        for (int i = 0; i < myAprilTagDetections.size(); i++) {
            AprilTagDetection myAprilTagDetection = myAprilTagDetections.get(i);

            if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                int myAprilTagIdCode = myAprilTagDetection.id;
                if (myAprilTagIdCode == idCode) {
                    aprilTagDetection = myAprilTagDetection;
                    myOpMode.telemetry.addData("y", myAprilTagDetection.ftcPose.y);
                    myOpMode.telemetry.addData("z", myAprilTagDetection.ftcPose.z);
                    myOpMode.telemetry.addData("roll", myAprilTagDetection.ftcPose.roll);
                    myOpMode.telemetry.addData("pitch", myAprilTagDetection.ftcPose.pitch);
                    myOpMode.telemetry.addData("yaw", myAprilTagDetection.ftcPose.yaw);
                }
            }
        }
        return aprilTagDetection;
    }

    public int getDefaultBlue() {
        return default_blue;
    }

    public int getDefaultRed() {
        return default_red;
    }
    public int getColorSensorBlue() {
        return colorSensor.blue();
    }

    public int getColorSensorRed() {
        return colorSensor.red();
    }

    //public int getColorSensorGreen() { return colorSensor.green(); }

    public int getDetectionSize() {
        return tagProcessor.getDetections().size();
    }

    //public int getColorSensorAlpha() { return colorSensor.alpha(); }

    //public int getColorSensorArgb() { return colorSensor.argb(); }

    /*
    // by Bo
    private void pickUpPixel() {
        clawPosition = Constants.clawOpen;
        servo2.setPosition(clawPosition);
        servo3.setPosition(clawPosition);
        sleep(100);
        wristPosition = Constants.wristDown;
        servo1.setPosition(wristPosition);
        sleep(1000);
        clawPosition = Constants.clawClose;
        servo2.setPosition(clawPosition);
        servo3.setPosition(clawPosition);
        sleep(1000);
        wristPosition = Constants.wristUp;
        servo1.setPosition(wristPosition);
    }

    private void releasePixel() {
        clawPosition = Constants.clawOpen;
        servo2.setPosition(clawPosition);
        servo3.setPosition(clawPosition);
        sleep(100);
        wristPosition = Constants.wristDown;
        servo1.setPosition(wristPosition);
        sleep(1000);
        clawPosition = Constants.clawClose;
        servo2.setPosition(clawPosition);
        servo3.setPosition(clawPosition);
        sleep(1000);
        wristPosition = Constants.wristUp;
        servo1.setPosition(wristPosition);
    }

    private int getLeftArmPosition() {
        return motor0core.getCurrentPosition();
    }

    private int getWristPosition() {
        return motor2core.getCurrentPosition();
    }
    */

    private void RotateP90() {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (90+currentAngle);
        double sign;
        while (Math.abs(diff)>2){
            sign = diff/Math.abs(diff);
            turn = sign*0.2; //turn = sign*0.3; // by Bo
            leftFront =  turn;
            rightFront = -turn;
            leftRear =  turn;
            rightRear =  - turn;

            motorHDLeftFront.setPower(leftFront);
            motorHDLeftRear.setPower(leftRear);
            motorHDRightFront.setPower(rightFront);
            motorHDRightRear.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (90+currentAngle);
            //telemetry.addData("current angle", currentAngle);
            //telemetry.addData("diff", diff);
            //telemetry.update();
        }
        motorHDLeftFront.setPower(0);
        motorHDLeftRear.setPower(0);
        motorHDRightFront.setPower(0);
        motorHDRightRear.setPower(0);
    }

    private void RotateM90() {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (90-currentAngle);
        double sign;
        while (Math.abs(diff)>2){
            sign = diff/Math.abs(diff);
            turn = -sign*0.2; //turn = -1*sign*0.3; // by Bo
            leftFront =  turn;
            rightFront = - turn;
            leftRear =  turn;
            rightRear = - turn;

            motorHDLeftFront.setPower(leftFront);
            motorHDLeftRear.setPower(leftRear);
            motorHDRightFront.setPower(rightFront);
            motorHDRightRear.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (90-currentAngle);
            //telemetry.addData("current angle", currentAngle);
            //telemetry.addData("diff", diff);
            //telemetry.update();
        }
        motorHDLeftFront.setPower(0);
        motorHDLeftRear.setPower(0);
        motorHDRightFront.setPower(0);
        motorHDRightRear.setPower(0);
    }
}


package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.deprecated.teamTeleOpCode.clawPosition;
//import static org.firstinspires.ftc.teamcode.deprecated.teamTeleOpCode.wristPosition;

import android.text.method.Touch;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.List;

public class MecanumRobot {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor motorHDLeftFront = null;
    private DcMotor motorHDLeftRear = null;
    private DcMotor motorHDRightFront = null;
    private DcMotor motorHDRightRear = null;

    private DcMotor motorCoreLeftArm = null;
    private DcMotor motorCoreRightArm = null;
    private DcMotor motorCoreSlides = null;
    private Servo servoWrist = null;
    private Servo servoLeftHand = null;
    private Servo servoRightHand = null;
    private Servo servoLauncher = null;

    public TouchSensor touchSensor = null;

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

        motorHDLeftFront = myOpMode.hardwareMap.get(DcMotor.class, "motor2hd");
        motorHDLeftRear = myOpMode.hardwareMap.get(DcMotor.class, "motor3hd");
        motorHDRightFront = myOpMode.hardwareMap.get(DcMotor.class, "motor0hd");
        motorHDRightRear = myOpMode.hardwareMap.get(DcMotor.class, "motor1hd");

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



        motorCoreLeftArm = myOpMode.hardwareMap.get(DcMotor.class, "motor0core");
        motorCoreRightArm = myOpMode.hardwareMap.get(DcMotor.class, "motor1core");
        motorCoreSlides = myOpMode.hardwareMap.get(DcMotor.class, "motor2core");

        //motorCoreSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorCoreLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        // RUN_WITHOUT_ENCODER mode - only set direction & power

        motorCoreLeftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCoreRightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCoreSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorCoreLeftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCoreRightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos

        servoWrist = myOpMode.hardwareMap.get(Servo.class, "servo0exp");
        servoLeftHand = myOpMode.hardwareMap.get(Servo.class, "servo1exp");
        servoRightHand = myOpMode.hardwareMap.get(Servo.class, "servo2exp");
        servoLauncher = myOpMode.hardwareMap.get(Servo.class, "servo3exp");

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

        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "sensorColorRangeR");
        colorSensor.enableLed(true);
        colorSensor2 = myOpMode.hardwareMap.get(ColorSensor.class, "sensorColor2");
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
        motorCoreLeftArm.setPower(powerScale);
        motorCoreRightArm.setPower(powerScale);

    }

    public int getMotorPositionLeftArm(){
        return motorCoreLeftArm.getCurrentPosition();
    }

    public int getMotorPositionSlide(){
        return motorCoreSlides.getCurrentPosition();
    }

    public void setMotorTargetSlide(int targetPosition) {
        motorCoreSlides.setTargetPosition(targetPosition);
        motorCoreSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setMotorPowerSlide(double powerScale) {

        motorCoreSlides.setPower(powerScale);



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


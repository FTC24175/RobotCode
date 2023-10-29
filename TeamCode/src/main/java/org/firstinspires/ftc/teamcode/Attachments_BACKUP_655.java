package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.teamTeleOpCode.wristPosition;
import static org.firstinspires.ftc.teamcode.teamTeleOpCode.clawPosition;
import static org.firstinspires.ftc.teamcode.teamTeleOpCode.iRobot;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Attachments {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDriveMotor, rightDriveMotor, leftArmMotor, rightArmMotor;
    public Servo clawServo;
    public static Servo  wristServo; //, camServo;
    public Rev2mDistanceSensor rightDistance, leftDistance, clawRightDistance, clawLeftDistance, clawDistance;

    public void initialize(HardwareMap hardwareMap) {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Motors operating the ARM
        leftDriveMotor = hardwareMap.dcMotor.get("motor1");
        rightDriveMotor = hardwareMap.dcMotor.get("motor2");

        // Motors operating the ARM
        leftArmMotor = hardwareMap.get(DcMotorEx.class,"motor3");
        rightArmMotor = hardwareMap.get(DcMotorEx.class, "motor4");

        // Servos operating wrist and claw
        clawServo =  hardwareMap.get(Servo.class, "servo1");
        wristServo = hardwareMap.get(Servo.class, "servo2");

        // Sensors
        // Todo: need to add

        // Motor Behavior
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /* --------------------------------------- ACCESSORS --------------------------------------- */
//    public double getRightDistance() {return rightDistance.getDistance(DistanceUnit.INCH);}
//    public double getLeftDistance() {return rightDistance.getDistance(DistanceUnit.INCH);}
//    public double getClawRightDistance() {return clawRightDistance.getDistance(DistanceUnit.INCH)}
//    public double getClawLeftDistance() {return clawLeftDistance.getDistance(DistanceUnit.INCH)}

    public double getClawDistance() {
        return clawDistance.getDistance(DistanceUnit.INCH);
    }

    /* ---------------------------------------- SETTERS ---------------------------------------- */
    public void runArmMotors(double power) {
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }
    public void setArmMotors(double power, int position) {
        leftArmMotor.setPower(power);
        leftArmMotor.setTargetPosition(position);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setPower(power);
        rightArmMotor.setTargetPosition(position);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void armDown() {
        wristPosition = Constants.wristOut;
        wristServo.setPosition(wristPosition);
        clawPosition = Constants.clawOpen;
        clawServo.setPosition(clawPosition);
        setArmMotors(0.3, 0);
    }
    public void pickUp() {
        wristPosition = Constants.wristIn;
        wristServo.setPosition(wristPosition);
        sleep(200);
        clawPosition = Constants.clawClose;
        clawServo.setPosition(clawPosition);
        sleep(500);
        wristPosition = Constants.wristOut;
        wristServo.setPosition(wristPosition);
    }

    public void setClawServo (double position) {
        clawServo.setPosition(position);
    }

    //    public void setCamServo (double position) {camServo.setPosition(position);}

    public void setWristServo(double position) {
        wristServo.setPosition(position);
    }

    public int getLeftArmPosition() {
        return leftArmMotor.getCurrentPosition();
    }

    public int getRightArmPosition() {
        return rightArmMotor.getCurrentPosition();
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    public double getWristPosition() {
        return wristServo.getPosition();
    }
}


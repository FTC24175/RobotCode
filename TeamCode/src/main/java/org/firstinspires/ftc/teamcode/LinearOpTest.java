package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="touchSensor")
public class LinearOpTest extends LinearOpMode {
    //public static Attachments iRobot = new Attachments();

    TouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException {
        //iRobot.initialize(hardwareMap);
        touch = hardwareMap.get(TouchSensor.class, "touch1");

        waitForStart();


        while (opModeIsActive()) {
            // If the touch sensor is pressed, stop the motor
            if (touch.isPressed()) {
                telemetry.addData("Path","Touch sensor is pressed");
                telemetry.update();

            } else { // Otherwise, run the motor
                telemetry.addData("Path","Waiting for touch sensor bumper to be pressed");
                telemetry.update();

            }
        }



    }
}

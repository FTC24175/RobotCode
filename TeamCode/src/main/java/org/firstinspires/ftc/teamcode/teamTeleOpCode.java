package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
Issues to fix:
Remap Controls
Add brake mode to motors so gravity doesnt affect the arm
Robot too fast: Add pressure sensitivity.
Simplify Buttons
Arm not moving up all the way
Make wrist move by hold the button instead of one press

Harware problems:
Attach control hub and other electronics to base
Labl Wires
add weight to front so it doesnt tip over when arm is out
Research Gear Ratio
zip ties are too long - cut off tails


Also scissor lift and lead screw should be use in case we ever plan on making the robot hang off a bar
 */

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class teamTeleOpCode extends OpMode {
    
    public static Attachments iRobot = new Attachments();

    private static double leftArmPosition = 0;
    private static double rightArmPosition = 0;

    // Servos
    public static double wristPosition = Constants.wristIn;
    public static double clawPosition = Constants.clawOpen;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        iRobot.initialize(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run after the driver hits PLAY but before they hit STOP
     * This is a continuous loop that responds to buttons being pressed on the game pad
     */
    @Override
    public void loop() {
        /* ------------------------------------ Drive ------------------------------------ */
        // Get Position constants
        leftArmPosition = iRobot.getLeftArmPosition();
        rightArmPosition = iRobot.getRightArmPosition();
        wristPosition  = iRobot.getWristPosition();
        clawPosition = iRobot.getClawPosition();

        // Motors
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        float motorPower = 0.3f;
        int liftMax = -1175;
        int liftMin = 0;
        int cycleLift1Pos = 0;

        /*
            Turn left or right when when left_stick is moved in x direction
         */
        if(rx > 0.1)
        {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.leftDriveMotor.setPower(rx);
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.rightDriveMotor.setPower(motorPower);
        }
        else if(rx < -0.1) {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.leftDriveMotor.setPower(Math.abs(rx));
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.rightDriveMotor.setPower(Math.abs(rx));
        }
        else
        {
            iRobot.leftDriveMotor.setPower(0);
            iRobot.rightDriveMotor.setPower(0);
        }

        /*
            Move forward or backward when left_stick is moved in y direction
         */
        if(ly > 0.1)
        {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.leftDriveMotor.setPower(motorPower);
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.rightDriveMotor.setPower(motorPower);
        }
        else if(ly < -0.1) {
            iRobot.leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.leftDriveMotor.setPower(motorPower);
            iRobot.rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.rightDriveMotor.setPower(rx);
        }
        else
        {
            iRobot.leftDriveMotor.setPower(0);
            iRobot.rightDriveMotor.setPower(0);
        }

        /*
            move wrist up and down when X & Y buttons are pressed
        */
        if (gamepad1.y)
        {
            wristPosition = Constants.wristIn;
            iRobot.wristServo.setPosition(wristPosition);
        }
        if (gamepad1.x)
        {
            wristPosition = Constants.wristOut;
            iRobot.wristServo.setPosition(wristPosition);
        }

        /*
            open and close claw when A and B buttons are pressed
        */
        if (gamepad1.a) {
            clawPosition = Constants.clawOpen;
            iRobot.clawServo.setPosition(clawPosition);
        } else if (gamepad1.b) {
            clawPosition = Constants.clawClose;
            iRobot.clawServo.setPosition(clawPosition);
        }

        /*
            Move arm up and down when right_stick is moved in y direction
        */
        if(ry > 0.1)
        {
            iRobot.leftArmMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.leftArmMotor.setPower(motorPower);
            iRobot.rightArmMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.rightArmMotor.setPower(motorPower);
        }
        else if(ry < -0.1) {
            iRobot.leftArmMotor.setDirection(DcMotor.Direction.REVERSE);
            iRobot.leftArmMotor.setPower(motorPower);
            iRobot.rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
            iRobot.rightArmMotor.setPower(motorPower);
        }
        else
        {
            iRobot.leftArmMotor.setPower(0);
            iRobot.rightArmMotor.setPower(0);
        }

        // Actually moves the claw and extension
        //iRobot.setClawServo(clawPosition);
        //iRobot.setArmServo(armPosition);

        /* ------------------------------------Arm Down ----------------------------------------*/
        if(gamepad1.left_trigger > 0.1) {
            iRobot.wristDown();
        }
        if(gamepad1.left_bumper) {
            iRobot.pickUpPixel();
        }

        /* ------------------------------------ Telemetry ------------------------------------ */
        // Telemetry is for debugging
        telemetry.addData("left arm position", leftArmPosition);
        telemetry.addData("right arm position", rightArmPosition);
        telemetry.addData("claw position", clawPosition);
        telemetry.addData("wrist position", wristPosition);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}

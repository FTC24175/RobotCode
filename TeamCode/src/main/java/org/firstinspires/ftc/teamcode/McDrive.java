package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "McDrive", group = "Iterative Opcode")
public class McDrive extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motor1");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("motor2");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("motor3");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("motor4");


            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            // Retrieve the IMU from the hardware map
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            double y, x, turn, theta, power;
            double sin, cos, max, leftFront, leftRear, rightFront, rightRear;

            waitForStart();

            if (isStopRequested()) return;


            while (opModeIsActive()) {
                y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                x = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x;

                if (gamepad1.a) {
                    RotateMinus90(imu, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                }
                if (gamepad1.x) {
                    movement(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor,0,0.5, 1600);
                    movement(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor,0,-0.5, 1600);
                    RotateMinus90(imu, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                    movement(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor,0,0.5, 4500);
                }

                if (gamepad1.y) {
                    movement(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor,0,0.5, 1600);
                    movement(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor,0,-0.5, 1600);
                    Rotate90(imu, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                    movement(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor,0,0.5, 4500);
                }

         /*       double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                telemetry.addData("current angle", botHeading*180/Math.PI);
                telemetry.update();*/

                if (gamepad1.b){
                    Rotate90(imu, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                }

                if (gamepad1.dpad_up) {
                    y = 0.35;
                    x = 0;
                } else if (gamepad1.dpad_down) {
                    y = -0.35;
                    x = 0;
                }
                if (gamepad1.dpad_left) {
                    x = -0.35;
                    y = 0;
                } else if (gamepad1.dpad_right) {
                    x = 0.35;
                    y = 0;
                }
                theta = Math.atan2(x,y);
                power = Math.sqrt(x*x + y*y);

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
         /*       if (gamepad1.options) {
                    imu.resetYaw();
                }*/

                sin = Math.sin(theta - Math.PI/4);
                cos = Math.cos(theta - Math.PI/4);
                max = Math.max(Math.abs(sin), Math.abs(cos));

                leftFront = power * cos/max + turn;
                rightFront = power * sin/max + turn;
                leftRear = power * sin/max - turn;
                rightRear = power * cos/max - turn;

                if ((power + Math.abs(turn))>1){
                    leftFront /= power + turn;
                    rightFront /= power + turn;
                    leftRear /= power + turn;
                    rightRear /= power + turn;
                }


                frontLeftMotor.setPower(leftFront);
                backLeftMotor.setPower(leftRear);
                frontRightMotor.setPower(rightFront);
                backRightMotor.setPower(rightRear);
            }
        }

        public void Rotate90(IMU imu, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
            imu.resetYaw(); // set to 0 degree
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            double leftFront, rightFront, leftRear, rightRear, turn;
            double diff =0;
            diff = (90+currentAngle);
            double sign;
            while (Math.abs(diff)>2){
                sign = diff/Math.abs(diff);
                turn = sign*0.2;
                leftFront =  turn;
                rightFront = turn;
                leftRear =  - turn;
                rightRear =  - turn;

                fl.setPower(leftFront);
                bl.setPower(leftRear);
                fr.setPower(rightFront);
                br.setPower(rightRear);

                currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                currentAngle = currentAngle *  180 / Math.PI;
                diff = (90+currentAngle);
                telemetry.addData("current angle", currentAngle);
                telemetry.addData("diff", diff);
                telemetry.update();
            }
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }

    public void RotateMinus90(IMU imu, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        imu.resetYaw(); // set to 0 degree
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentAngle = currentAngle *  180 / Math.PI;
        double leftFront, rightFront, leftRear, rightRear, turn;
        double diff =0;
        diff = (90-currentAngle);
        double sign;
        while (Math.abs(diff)>2){
            sign = diff/Math.abs(diff);
            turn = -sign*0.2;
            leftFront =  turn;
            rightFront = turn;
            leftRear =  - turn;
            rightRear =  - turn;

            fl.setPower(leftFront);
            bl.setPower(leftRear);
            fr.setPower(rightFront);
            br.setPower(rightRear);

            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            currentAngle = currentAngle *  180 / Math.PI;
            diff = (90-currentAngle);
            telemetry.addData("current angle", currentAngle);
            telemetry.addData("diff", diff);
            telemetry.update();
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    public void movement(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, double targetX, double targetY, long time){
        double leftFront, rightFront, leftRear, rightRear;

        double x = targetX;
        double y = targetY;
        double theta = Math.atan2(x,y);
        double power = Math.sqrt(x*x + y*y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        leftFront = power * cos/max;
        rightFront = power * sin/max;
        leftRear = power * sin/max;
        rightRear = power * cos/max;

        fl.setPower(leftFront);
        bl.setPower(leftRear);
        fr.setPower(rightFront);
        br.setPower(rightRear);

        sleep(time);

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }

}


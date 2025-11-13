package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config
public class Drivetrain {
    //declare motors
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public IMU imu;

    //botHeading used for field centric drive
    private double botHeading;

    //constants used for tuning auto alignment
    public static double AUTO_ALIGN_MAX_SPEED = 0.4; //auto alignment speed is clipped to minimum negative this and maximum positive this (bilateral tolerance)
    public static double AUTO_ALIGN_GAIN = 0.02; //converts degrees to power, at a 1:100 ratio (ex: 25 degrees = 0.25 power)

    public static double GROUNDING_POWER = 0.5;
    private OpMode opMode;


    //constructor which acts as an initialization function for whenever an object of the class is created
    public Drivetrain(OpMode linearopmode)
    {
        this.opMode = linearopmode;
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontleft");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backleft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontright");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backright");
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        // adjust the orientation parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // actually set the parameters
        imu.initialize(parameters);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void updateIMU() {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    //main mecanum drive function
    public void drive(Gamepad gamepad)
    {
        //get gamepad inputs
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); //always offset the robot's heading to only move in the four cardinal directions
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1; //counter imperfect strafing

        //speed control with triggers
        double maxSpeed = 0.5;
        double maxSpeedMultiplier;
        maxSpeedMultiplier = maxSpeed + ((-gamepad.right_trigger * (maxSpeed * 0.5)) + (gamepad.left_trigger * maxSpeed));

        //solve for power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        //multiply calculated power with the speed control to obtain final power for the motors
        frontLeft.setPower(frontLeftPower * maxSpeedMultiplier);
        backLeft.setPower(backLeftPower * maxSpeedMultiplier);
        frontRight.setPower(frontRightPower * maxSpeedMultiplier);
        backRight.setPower(backRightPower * maxSpeedMultiplier);
    }
    public void driveAutoAlign(double x, double y, double rotation)
    {
        //get gamepad inputs
        double ypower = -y * 0.25; // lower the power when auto aligning
        double xpower = x * 0.26; //multiplied by an extra 0.1 to counter imperfect strafing
        double rotationpower = rotation;

        //solve for power
        double denominator = Math.max(Math.abs(ypower) + Math.abs(xpower) + Math.abs(rotationpower), 1);
        double frontLeftPower = (ypower + xpower + rotationpower) / denominator;
        double backLeftPower = (ypower - xpower + rotationpower) / denominator;
        double frontRightPower = (ypower - xpower - rotationpower) / denominator;
        double backRightPower = (ypower + xpower - rotationpower) / denominator;

        //multiply calculated power with the speed control to obtain final power for the motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public double calculateAutoAlignPower(double bearing) {
        double autoAlignPower;
        autoAlignPower = Range.clip(bearing * AUTO_ALIGN_GAIN, -AUTO_ALIGN_MAX_SPEED, AUTO_ALIGN_MAX_SPEED);
        return autoAlignPower;
    }
    public void grounder(){
        frontLeft.setPower(GROUNDING_POWER);
        backLeft.setPower(-GROUNDING_POWER);
        frontRight.setPower(GROUNDING_POWER);
        backRight.setPower(-GROUNDING_POWER);
    }
//unused and unfinished drivetrain methods
//    public void forward() {
//            frontRight.setPower(1);
//            backRight.setPower(1);
//            frontLeft.setPower(1);
//            backLeft.setPower(1);
//        }
//    public void backward(){
//            frontRight.setPower(-1);
//            backRight.setPower(-1);
//            frontLeft.setPower(-1);
//            backLeft.setPower(-1);
//        }
//    public void leftStrafe(int direction) {
//        frontRight.setPower(1);
//        backRight.setPower(-1);
//        frontLeft.setPower(1);
//        backLeft.setPower(-1);
//    }
//    public void rightStrafe(int direction){
//        frontRight.setPower(1);
//        backRight.setPower(-1);
//        frontLeft.setPower(1);
//        backLeft.setPower(-1);
//    }
}



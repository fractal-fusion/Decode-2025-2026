package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config
public class Drivetrain {
    //declare motors
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor testWheel;
    public Servo tiltLeft;
    public Servo tiltRight;
    public static double TILT_RIGHT_SERVO_OFFSET = 0; //offset for tilt since the servo teeth are a little off
    public static double TILT_OFF_POSITION = 0;
    public static double TILT_ON_POSITION = 0.2;

    public static String testWheelHardwareMapName = "frontright";
    public IMU imu;

    //botHeading used for field centric drive
    private double botHeading;

    //constants used for tuning auto alignment
    public static double AUTO_ALIGN_MAX_SPEED = 0.8; //auto alignment speed is clipped to minimum negative this and maximum positive this (bilateral tolerance)
    public static double AUTO_ALIGN_GAIN_LIMELIGHT = 0.0175; //converts tx from limelight to power
    public static double AUTO_ALIGN_GAIN_ODO = 0.85; //converts odometry bearing to power
    public static double SECONDARY_PID_THRESHOLD = 0.2; //less than this switch to secondary pid
    public static double AUTO_ALIGN_INTEGRAL_ODO = 0.005;
    public static double ODO_HEADING_VALID_RANGE = 1.5;
    public static double RELOCALIZATION_VELOCITY_THRESHOLD = 0.1;

    public static double P = 0.535;
    public static double I = 0;
    public static double D = 0;

    public static double SECONDARY_P = 1.65;
    public static double SECONDARY_I = 0;
    public static double SECONDARY_D = 0;
    public Timer headingPIDTimer = new Timer();
    public Timer secondaryHeadingPIDTimer = new Timer();
    public double integralSum = 0;

    public double lastError = 0;

    public double secondaryIntegralSum = 0;

    public double secondaryLastError = 0;

    public static double AUTO_ALIGN_DRIVE_POWER_MULTIPLIER_MIDPOINT = 0.45; //half of max power
    public static double IS_FAR_THRESHOLD_Y = 40; //less than this Y value is considered far

    public static double FUTURE_VELOCITY_TIME = 0.15;

    public Gamepad currentGamepad = new Gamepad();
    public Gamepad previousGamepad = new Gamepad();
    public boolean grounded = false;
    public boolean isFollowing = false; //follower.isbusy doesn't work so this boolean will make sure hold pose or follow path is only called once
    public boolean tilted = false;
    public Pose holdPose = new Pose();
//    private Follower follower;

//        ColorDetector colorDetector = new ColorDetector(this);
    private OpMode opMode;


    //constructor which acts as an initialization function for whenever an object of the class is created
    public Drivetrain(OpMode linearopmode)
    {
        this.opMode = linearopmode;
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontleft");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backleft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontright");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backright");

//        tiltLeft = opMode.hardwareMap.get(Servo.class, "tiltleft");
//        tiltRight = opMode.hardwareMap.get(Servo.class, "tiltright");
//
//        tiltLeft.setDirection(Servo.Direction.FORWARD);
//        tiltRight.setDirection(Servo.Direction.FORWARD);
//
//        setTiltPosition(0);

        testWheel = opMode.hardwareMap.get(DcMotor.class, testWheelHardwareMapName);

        imu = opMode.hardwareMap.get(IMU.class, "imu");


//        follower = Constants.createFollower(opMode.hardwareMap);
//        follower.setStartingPose(PoseStorage.currentPose == null ? new Pose() : PoseStorage.currentPose);

        // adjust the orientation parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // actually set the parameters
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void updateIMU() {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void testWheel(){
        testWheel.setPower(0.25);
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
    public void driveAutoAlign(Gamepad gamepad, double rotation)
    {

        double maxSpeed = AUTO_ALIGN_DRIVE_POWER_MULTIPLIER_MIDPOINT;
        double maxSpeedMultiplier;
        maxSpeedMultiplier = maxSpeed + ((-gamepad.right_trigger * (maxSpeed * 0.5)) + (gamepad.left_trigger * maxSpeed));

        //get gamepad inputs
        double ypower = -gamepad.left_stick_y * maxSpeedMultiplier; // lower the power when auto aligning
        double xpower = gamepad.left_stick_x * (maxSpeedMultiplier + 0.01); //multiplied by an extra 0.1 to counter imperfect strafing
        double rotationpower = rotation;

        double rotX = xpower * Math.cos(-botHeading) - ypower * Math.sin(-botHeading); //always offset the robot's heading to only move in the four cardinal directions
        double rotY = xpower * Math.sin(-botHeading) + ypower * Math.cos(-botHeading);
        rotX = rotX * 1.1; //counter imperfect strafing

        //solve for power
        double denominator = Math.max(Math.abs(ypower) + Math.abs(xpower) + Math.abs(rotationpower), 1);
        double frontLeftPower = (rotY + rotX + rotationpower) / denominator;
        double backLeftPower = (rotY - rotX + rotationpower) / denominator;
        double frontRightPower = (rotY - rotX - rotationpower) / denominator;
        double backRightPower = (rotY + rotX - rotationpower) / denominator;

        //multiply calculated power with the speed control to obtain final power for the motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

//    public void setTiltPosition(double position) {
//        tiltLeft.setPosition(position);
//        tiltRight.setPosition(position + TILT_RIGHT_SERVO_OFFSET);
//    }

    public double calculateAutoAlignPowerLimelight(double bearing) {
        return Range.clip(bearing * AUTO_ALIGN_GAIN_LIMELIGHT, -AUTO_ALIGN_MAX_SPEED, AUTO_ALIGN_MAX_SPEED);
    }

    public double calculatePrimaryPIDAutoAlignPowerOdo(double error) {
        double derivative = error - lastError;

        integralSum = integralSum + (error * headingPIDTimer.getElapsedTimeSeconds());

        double power = (P * error) + (I * integralSum) + (D * derivative);

        lastError = error;

        headingPIDTimer.resetTimer();

        return power;
    }

    public double calculateSecondaryPIDAutoAlignPowerOdo(double error) {
        if (Math.abs(error) > SECONDARY_PID_THRESHOLD){ //send power when not between -deadzone or +deadzone
            double derivative = error - lastError;

            integralSum = integralSum + (error * headingPIDTimer.getElapsedTimeSeconds());

            double power = (P * error) + (I * integralSum) + (D * derivative);

            lastError = error;

            headingPIDTimer.resetTimer();

            return power;
        }
        else{
            double derivative = error - secondaryLastError;

            secondaryIntegralSum = secondaryIntegralSum + (error * secondaryHeadingPIDTimer.getElapsedTimeSeconds());

            double power = (SECONDARY_P * error) + (SECONDARY_I * secondaryIntegralSum) + (SECONDARY_D * derivative);

            secondaryLastError = error;

            secondaryHeadingPIDTimer.resetTimer();

            return power;
        }
    }

//    public void updateIntegralSum(double bearing, double seconds) {
//        integralSum += bearing * seconds;
//    }

    public double calculateOdoGoalAngle(Pose robotPose, Pose goalPose) {
        double vectorY = goalPose.getY() - robotPose.getY();
        double vectorX = goalPose.getX() - robotPose.getX();

        double angle = Math.atan2(vectorY, vectorX);
        if (angle < 0) angle += Math.PI * 2;

        return angle;
    }
    public double calculateOdoGoalBearing(Pose robotPose, Pose goalPose) {
        double vectorY = goalPose.getY() - robotPose.getY();
        double vectorX = goalPose.getX() - robotPose.getX();

        double angle = (Math.atan2(vectorY, vectorX) - robotPose.getHeading());
        if (angle <= -Math.PI) angle += Math.PI * 2;
        if (angle > Math.PI) angle -= Math.PI * 2;
        return angle;
    }

    public double calculateOdoGoalDistance(Pose robotPose, Pose goalPose){
        double vectorY = Math.abs(goalPose.getY() - robotPose.getY());
        double vectorX = Math.abs(goalPose.getX() - robotPose.getX());

        return Math.hypot(vectorX, vectorY);
    }

    public double calculateAirTime(double distance){
        return (0.0025 * distance + 0.3871);
    }

    public Pose calculateVirtualGoalPose(Follower follower, double airtime, Pose goalpose) {
        double ballDistanceFromGoal = follower.getVelocity().getMagnitude() * airtime; //calculate how the distance the ball will travel in the air, using relation of distance vs time since longer time means longer distance
                                                                                       //velocity is inches/second, so need to multiply by time to get actual distance

        Vector futureRobotOffset = new Vector(ballDistanceFromGoal, follower.getVelocity().getTheta());
        return new Pose(-futureRobotOffset.getXComponent() + goalpose.getX(), -futureRobotOffset.getYComponent() + goalpose.getY());
    }

    public boolean isFarOdometry(Pose currentPose){
        return currentPose.getY() < IS_FAR_THRESHOLD_Y;
    }

    public boolean isSlowForRelocalization(Follower follower){
        return follower.getVelocity().getMagnitude() < RELOCALIZATION_VELOCITY_THRESHOLD;
    }

    public boolean ifSlowDriverOdometry(double distance){
        if (distance > 110) {
            return true;
        }
        else if (distance < 28) {
            return true;
        }

        return false;
    }
//    public void toggleGrounded() {
//        if (currentGamepad.b && !previousGamepad.b) {
//            grounded = !grounded;
//        }
//
//        //control the claw based on the boolean
//        if (grounded) {
//            follower.holdPoint(holdPose);
//        }
//        else {
//            holdPose = follower.getPose();
//            follower.breakFollowing();
//        }
//    }
//
//    public void toggleTilt(Gamepad gamepad) {
//        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
//            tilted = !tilted;
//        }
//
//        //control the tilt based on the boolean
//        if (tilted) {
//            setTiltPosition(TILT_ON_POSITION);
//        }
//        else {
//            setTiltPosition(TILT_OFF_POSITION);
//        }
//    }
//
//    public void updateGamepad(Gamepad gamepad) { //debounce method
//        previousGamepad.copy(currentGamepad);
//
//        currentGamepad.copy(gamepad);
//    }

//    public void grounder(){
//        frontLeft.setPower(GROUNDING_POWER);
//        backLeft.setPower(-GROUNDING_POWER);
//        frontRight.setPower(GROUNDING_POWER);
//        backRight.setPower(-GROUNDING_POWER);
//    }
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



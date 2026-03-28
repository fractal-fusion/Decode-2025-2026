package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Intake {
    public DcMotor intakeBack;
    public DcMotor intakeFront;
//    public Servo flicker;
    private OpMode opMode;

    //static variables for positions of the flicker servo
    public static final double FLICKER_OPEN_POSITION = 0;
    public static final double FLICKER_CLOSE_POSITION = 0.6;
    public static final double FLICKER_HOLD_POSITION = 0.4;
    public static double DRIVER_INTAKE_POWER = 1;
    public static double DRIVER_CLOSE_SHOOTING_POWER = 1;
    public static double DRIVER_FAR_SHOOTING_POWER = 0.6;
    public static double DRIVER_CLOSE_SLOW_SHOOTING_POWER = 0.7;
    public double driverPower = DRIVER_CLOSE_SHOOTING_POWER;
    public static double AUTO_DRIVER_POWER_CLOSE = 1;
    public static double AUTO_DRIVER_POWER_FAR = 0.7;

//    public static final double FLICKER_CYCLE_POSITION = 0.25;

    //gamepads for rising edge detector
    public Gamepad currentGamepad = new Gamepad();
    public Gamepad previousGamepad = new Gamepad();
//    private boolean flickerIsOpen = true;

    //variables for setting flicker time
    private ElapsedTime timer;
    private double currentFlickerTime;

    public Intake(LinearOpMode linearOpMode, double initPosition){
        this.opMode = linearOpMode;
        timer = new ElapsedTime();

        intakeBack = opMode.hardwareMap.get(DcMotor.class, "intakeback");
        intakeFront = opMode.hardwareMap.get(DcMotor.class, "intakefront");
//        flicker = opMode.hardwareMap.get(Servo.class, "flicker");

        //intake.setDirection(DcMotor.Direction.REVERSE); // reverse direction so positive is intake, negative is outtake
        intakeFront.setDirection(DcMotor.Direction.FORWARD);
        intakeBack.setDirection(DcMotor.Direction.REVERSE);

//        flicker.setDirection(Servo.Direction.REVERSE); //reverse flicker servo so increasing to position 1 is flick inside
//        flicker.setPosition(initPosition); //open flicker on initialization
    }

    public void turnOnIntake(){
        intakeFront.setPower(driverPower);
        intakeBack.setPower(driverPower);
    }

    public void setDriverPower(double power){
        driverPower = power;
    }

    public double calculateDriverPower(double ta) {
        return Range.clip(0.1 * ta + 0.7, 0.8, 1);
    }
    public void turnOnIntakeAuto(){
        intakeFront.setPower(AUTO_DRIVER_POWER_CLOSE);
        intakeBack.setPower(AUTO_DRIVER_POWER_CLOSE);
    }

    public void turnOnIntakeAutoFar(){
        intakeFront.setPower(AUTO_DRIVER_POWER_FAR);
        intakeBack.setPower(AUTO_DRIVER_POWER_FAR);
    }
//    public void turnOnDriverSlow(){
//        intakeFront.setPower(0);
//        intakeBack.setPower(0.5);
//    }

    public void turnOnOuttake(){
        intakeFront.setPower(-1);
        intakeBack.setPower(-1);
    }

    public void turnOffIntake(){
        intakeFront.setPower(0);
        intakeBack.setPower(0);
    }

//    public void resetFlickerOpenTimer(){ //turn on flicker and turn it off after a certain amount of time
//        timer.reset();
//    }

//    public void updateFlickerOpenTimer(){
//        currentFlickerTime = timer.time();
//    }

//    public void checkFlickerOpenTimer(Gamepad gamepad) {
//        if (currentFlickerTime > 1 && !gamepad.dpad_left) { //after a second open the flicker
//            flicker.setPosition(FLICKER_OPEN_POSITION);
//            flickerIsOpen = true;
//        }
//    }

//    public void toggleFlicker() {
//        if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
//            flickerIsOpen = !flickerIsOpen;
//        }
//
//        //control the claw based on the boolean
//        if (flickerIsOpen) {
//            flicker.setPosition(FLICKER_OPEN_POSITION);
//        }
//        else {
//            flicker.setPosition(FLICKER_CLOSE_POSITION);
//        }
//    }
    public void updateGamepad(Gamepad gamepad) { //debounce method
        previousGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad);
    }

//    public void setFlickerPosition(double position){
//        flicker.setPosition(position);
//    }

    //runnables for auto

//    public void closeFlicker(){
//        flicker.setPosition(FLICKER_CLOSE_POSITION);
//    }
//
//    public void openFlicker(){
//        flicker.setPosition(FLICKER_OPEN_POSITION);
//    }
//    public void holdFlicker(){
//        flicker.setPosition(FLICKER_HOLD_POSITION);
//    }


}

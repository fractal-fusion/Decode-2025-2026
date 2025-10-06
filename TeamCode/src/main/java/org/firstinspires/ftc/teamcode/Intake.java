package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    public DcMotor driver;
    public DcMotor intake;
    public Servo flicker;
    private OpMode opMode;

    //static variables for positions of the flicker servo
    public static final double FLICKER_OPEN_POSITION = 0;
    public static final double FLICKER_CLOSE_POSITION = 0.6;

    //gamepads for rising edge detector
    public Gamepad currentGamepad = new Gamepad();
    public Gamepad previousGamepad = new Gamepad();
    private boolean flickerIsOpen = true;

    //variables for setting flicker time
    private ElapsedTime timer;
    private double currentTime;

    public Intake(LinearOpMode linearOpMode){
        this.opMode = linearOpMode;
        timer = new ElapsedTime();

        driver = opMode.hardwareMap.get(DcMotor.class, "driver");
        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
        flicker = opMode.hardwareMap.get(Servo.class, "flicker");

        intake.setDirection(DcMotor.Direction.REVERSE); // reverse direction so positive is intake, negative is outtake

        flicker.setDirection(Servo.Direction.REVERSE); //reverse flicker servo so increasing to position 1 is flick inside
    }

    public void activateFlickerOpenTimer(){ //turn on flicker and turn it off after a certain amount of time
        timer.reset();
        currentTime = timer.time();
    }

    public void checkFlickerOpenTimer() {
        if (currentTime > currentTime + 500 && flicker.getPosition() == FLICKER_CLOSE_POSITION) { //after 500 milliseconds open the flicker
            flicker.setPosition(FLICKER_OPEN_POSITION);
        }
    }

    public void toggleFlicker() {
        if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
            flickerIsOpen = !flickerIsOpen;
        }

        //control the claw based on the boolean
        if (flickerIsOpen) {
            flicker.setPosition(FLICKER_OPEN_POSITION);
        }
        else {
            flicker.setPosition(FLICKER_CLOSE_POSITION);
        }
    }
    public void updateGamepad(Gamepad gamepad) { //debounce method
        previousGamepad.copy(currentGamepad);

        currentGamepad.copy(gamepad);
    }

    public void flickerPosition(double position){
        flicker.setPosition(position);
    }

}

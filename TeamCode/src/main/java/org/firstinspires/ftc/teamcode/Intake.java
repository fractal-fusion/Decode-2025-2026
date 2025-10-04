package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor driver;
    public DcMotor intake;
    public Servo flicker;
    private OpMode opMode;

    public Intake(LinearOpMode linearOpMode){
        this.opMode = linearOpMode;

        driver = opMode.hardwareMap.get(DcMotor.class, "driver");
        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
        flicker = opMode.hardwareMap.get(Servo.class, "flicker");
        intake.setDirection(DcMotor.Direction.REVERSE); // reverse direction so positive is intake, negative is outtake

        flicker.setDirection(Servo.Direction.REVERSE); //reverse flicker servo so increasing to position 1 is flick inside
    }

    public void intake(){ //skeleton methods
        //raise pitch, turn on driver and intake
    }

    public void flickerPosition(double position){
        flicker.setPosition(position);
    }

}

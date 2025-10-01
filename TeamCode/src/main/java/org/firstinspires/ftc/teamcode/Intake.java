package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor driver;
    public DcMotor intake;
    public Servo flicker;
    private OpMode opMode;

    public Intake(LinearOpMode linearOpMode){
        this.opMode = linearOpMode;

//        driver = opMode.hardwareMap.get(DcMotor.class, "driver");
//        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
        flicker = opMode.hardwareMap.get(Servo.class, "flicker");
//        intake.setDirection(DcMotor.Direction.REVERSE); // reverse direction so positive is intake, negative is outtake
    }

    public void intake(){ //skeleton methods
        intake.setPower(1);
    }
    public void driver(){
        driver.setPower(1);

    }
    public void flickerPos(double pos){
        flicker.setPosition(pos);
    }

}

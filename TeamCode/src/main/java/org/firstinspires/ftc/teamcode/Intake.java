package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    public DcMotor driver;
    public DcMotor intake;
    private OpMode opMode;

    public Intake(LinearOpMode linearOpMode){
        this.opMode = linearOpMode;

        driver = opMode.hardwareMap.get(DcMotor.class, "driver");
        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
    }
}

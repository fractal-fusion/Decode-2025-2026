package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    public DcMotor driver;
    public DcMotor intake;

    public void intake(){
        intake.setPower(1);
    }
    public void drive(){
        driver.setPower(1);
    }
}

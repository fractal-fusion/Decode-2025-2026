package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorDetector {

    public ColorSensor colorSensor;
    private OpMode Opmode;
    public ColorDetector(OpMode linearopmode) {
        this.Opmode = linearopmode;
        colorSensor = Opmode.hardwareMap.get(ColorSensor.class, "color");
    }
}

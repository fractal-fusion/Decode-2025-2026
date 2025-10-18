package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorDetector {

    public ColorSensor colorSensor;
    private OpMode Opmode;
    int R;
    int G;
    int B;

    public ColorDetector(OpMode linearopmode) {
        this.Opmode = linearopmode;
        colorSensor = Opmode.hardwareMap.get(ColorSensor.class, "color");
    }
    public int[] getColor(){
        R = colorSensor.red();
        G = colorSensor.green();
        B = colorSensor.blue();
        int[] RGB = {R, G, B};
        return RGB;
    }
    public void telemetryColors(){
        Opmode.telemetry.addData("Red:", colorSensor.red());
        Opmode.telemetry.addData("Green:", colorSensor.green());
        Opmode.telemetry.addData("Blue:", colorSensor.blue());
        Opmode.telemetry.update();
    }

}

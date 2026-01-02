package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IndicatorLight {
    private OpMode opMode;
    private Servo indicatorLight;
    public static double INDICATOR_LIGHT_RED = 0.722;
    public static double INDICATOR_LIGHT_GREEN = 0.500;
    private double currentColor = 0.0;

    public IndicatorLight(OpMode linearopmode){
        this.opMode = linearopmode;
        indicatorLight = opMode.hardwareMap.get(Servo.class, "light");
    }

    public void setIndicatorLight(double color){
        indicatorLight.setPosition(color);
    }

    public void setRGB() {
        indicatorLight.setPosition(0.277 + (currentColor % 0.723));
        currentColor += 0.0005;
    }
}

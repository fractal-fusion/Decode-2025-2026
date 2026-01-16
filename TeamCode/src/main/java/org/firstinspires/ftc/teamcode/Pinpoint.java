package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Pinpoint {
    private OpMode opMode;
    public GoBildaPinpointDriver pinpoint;
    public Pinpoint(OpMode linearopmode){
        opMode = linearopmode;

        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void resetPinpoint(){
        pinpoint.resetPosAndIMU();
    }
}

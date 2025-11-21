package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Test")
public class LimelightTestOpmodeModular extends LinearOpMode {

    @Override
    public void runOpMode() {

        Limelight limelight = new Limelight(this);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            limelight.telemetryAprilTag();
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="testShooterOpmode", group="Robot")
public class testShooterOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(this);
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            shooter.testShoot(gamepad1);
            telemetry.addData("power", shooter.shootPower);
            telemetry.update();
        }
    }
}
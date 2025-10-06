package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="productionOpmode", group="Robot")
public class productionOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this);
        Shooter shooter = new Shooter(this);
        Camera camera = new Camera(this, 3);


        camera.setExposure(6); //low exposure and high gain to reduce blur for autoalignment
        camera.setGain(250);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();



        while (opModeIsActive())
        {
            //update the imu with the rotation of the robot
            drivetrain.updateIMU();
            //drivetrain controls (field centric drive + autoalignment)
            if (gamepad1.a) {
//                camera.turnOnCamera();
                drivetrain.driveAutoAlign(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.calculateAutoAlignPower(camera.getBearing()));
            }
            else {
//                camera.turnOffCamera();
                drivetrain.drive(gamepad1);
            }

            if (gamepad1.x) {
                drivetrain.resetIMU();
            }
        }
    }
}

//TODO: make the pitch only go to zero when the shooter is at the right velocity, create cycling control
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="productionOpmode", group="Robot")
public class productionOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this);
        Shooter shooter = new Shooter(this);
        Intake intake = new Intake(this);
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
            //update the gamepad2 states of the shooter object for the rising edge detector to work
            shooter.updateGamepad(gamepad2);
            //update the gamepad2 states of the intake object for the rising edge detector to work
            intake.updateGamepad(gamepad2);

            //drivetrain controls (field centric drive + autoalignment)
            if (gamepad1.a) {
                drivetrain.driveAutoAlign(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.calculateAutoAlignPower(camera.getBearing()));
            }
            else if (gamepad1.x) {
                drivetrain.resetIMU();
            }
            else {
                drivetrain.drive(gamepad1);
            }

            //mechanism intake control
            if(gamepad2.a){
                intake.turnOnIntake();
                shooter.setPitchPosition(Shooter.PITCH_INTAKE_POSITION);
            }
            else if (gamepad2.b){
                intake.turnOnOuttake();
            }
            else {
                intake.turnOffIntake();
            }
            //mechanism intake flicker control
            if(gamepad2.dpad_left) {
                intake.toggleFlicker();
                intake.activateFlickerOpenTimer();
            }
            intake.checkFlickerOpenTimer(); //automatically open flicker after 500ms if its closed

            //mechanism shooter control
            if (gamepad2.x) {
                shooter.toggleShooter();
            }
            //flatten the pitch when scoring so balls can pass to shooter motors
            if (shooter.shooterAtTargetVelocity()){
                shooter.setPitchPosition(0); //flatten the pitch when scoring so balls can pass to shooter motors
            }

        }
    }
}

//TODO: make the pitch only go to zero when the shooter is at the right velocity, create cycling control
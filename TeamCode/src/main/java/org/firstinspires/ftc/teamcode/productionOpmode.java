package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="productionOpmode", group="Robot")
public class productionOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this);
        Shooter shooter = new Shooter(this);
        Intake intake = new Intake(this, Intake.FLICKER_OPEN_POSITION);
        Camera camera = new Camera(this, 3);
//        ColorDetector colorDetector = new ColorDetector(this);

        camera.setExposure(6); //low exposure and high gain to reduce blur for autoalignment
        camera.setGain(250);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

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
                drivetrain.driveAutoAlign(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.calculateAutoAlignPower(-camera.getBearing()));
            }
            else if (gamepad1.x) {
                drivetrain.resetIMU();
            }
            else {
                drivetrain.drive(gamepad1);
            }

            //mechanism intake control
            if(gamepad2.a) {
                intake.turnOnIntake();
                if (!shooter.shooterAtTargetVelocity()) {
                    shooter.setPitchPosition(Shooter.PITCH_INTAKE_POSITION);
                }
            }
            else if (gamepad2.b){
                intake.turnOnOuttake();
            }
            else if (gamepad2.right_bumper){
                shooter.cycling = true;
                shooter.setPitchPosition(Shooter.PITCH_CYCLE_POSITION);
//                shooter.setRampPosition(Shooter.RAMP_CYCLE_POSITION);
                intake.turnOnIntake();
                shooter.turnOnShooter(Shooter.CYCLING_RPM);
//                intake.setFlickerPosition(Intake.FLICKER_CYCLE_POSITION);
            }
            else {
                intake.turnOffIntake();
                shooter.cycling = false;
                if (!shooter.on) { //make sure shooter is off after cycling and not shooting
                    shooter.turnOffShooter();
//                    shooter.setRampPosition(0);
                }
            }
            //mechanism intake flicker control
            if(gamepad2.dpad_left) {
                intake.toggleFlicker();
                intake.resetFlickerOpenTimer();
            }

            intake.updateFlickerOpenTimer(); //update the timer with the current time
            intake.checkFlickerOpenTimer(gamepad2); //automatically open flicker after a second if its closed, unless the gamepad button is held down

            //mechanism shooter control
            if (gamepad2.x) {
                camera.setHeadingOffset(Camera.HEADING_OFFSET_CLOSE); //TODO: automate this by getting the distance
                shooter.toggleShooterClose();
            }
            else if (gamepad2.y){
                camera.setHeadingOffset(Camera.HEADING_OFFSET_FAR);
                shooter.toggleShooterFar();
            }
            //flatten the pitch when scoring so balls can pass to shooter motors
            if (shooter.shooterAtTargetVelocity() && !shooter.cycling && shooter.checkPitchDebounceTimer()){
                shooter.setPitchPosition(Shooter.PITCH_SCORE_POSITION);
                shooter.resetPitchTimer(); //reset the debounce
            }
            else if (!shooter.shooterAtTargetVelocity() && !shooter.cycling){
                shooter.setPitchPosition(Shooter.PITCH_INTAKE_POSITION); //automatically raise the pitch when not ready to shoot
            }

            shooter.updatePitchDebounceTimer(); //update the debounce timer

//            telemetry.addData("intake current time:", intake.currentTime);
            telemetry.addData("shooter left velocity:", shooter.shooterLeftGetVelocity());
            telemetry.addData("shooter right velocity:", shooter.shooterRightGetVelocity());
            telemetry.addData("shooter at velocity:", shooter.shooterAtTargetVelocity());

//            telemetry.addData("shooter at velocity:", shooter.shooterAtTargetVelocity());
            telemetry.addData("apriltag bearing:", camera.getBearing());
            telemetry.addData("drive power:", drivetrain.calculateAutoAlignPower(camera.getBearing()));
            telemetry.update();
        }
    }
}


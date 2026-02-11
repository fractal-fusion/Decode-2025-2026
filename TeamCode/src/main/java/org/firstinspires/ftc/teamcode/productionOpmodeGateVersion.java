package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="productionOpmodeGateVersion", group="Robot")
public class productionOpmodeGateVersion extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        Drivetrain drivetrain = new Drivetrain(this);
        Shooter shooter = new Shooter(this);
        Intake intake = new Intake(this, Intake.FLICKER_OPEN_POSITION);
//         Camera camera = new Camera(this, 3);
        Limelight limelight = new Limelight(this);
        IndicatorLight indicatorLight = new IndicatorLight(this);
        Follower follower;

//        ColorDetector colorDetector = new ColorDetector(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseManager.currentPose == null ? new Pose() : PoseManager.currentPose); //get pose handed off from auto otherwise just create a new one

//        camera.setExposure(6); //low exposure and high gain to reduce blur for autoalignment not needed
//        camera.setGain(250);

        shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            //update follower
            follower.update();
            //update the imu with the rotation of the robot
            drivetrain.updateIMU();
            //update the gamepad2 states of the shooter object for the rising edge detector to work
            shooter.updateGamepad(gamepad2);
            //update the gamepad2 states of the intake object for the rising edge detector to work
            intake.updateGamepad(gamepad2);

            //drivetrain controls (field centric drive + autoalignment)
            if (gamepad1.right_bumper){
                drivetrain.grounded = true;
            }
            else if (gamepad1.b) {
                drivetrain.driveAutoAlign(gamepad1, drivetrain.calculateAutoAlignPowerLimelight(limelight.getBearing()));
                drivetrain.holdPose = follower.getPose();

                if (!new Pose().roughlyEquals(limelight.getRobotPose(), 1)){ //recalibrate pose using limelight TODO: relocalize limelight
                    follower.setPose(limelight.getRobotPose());
                }
            }
            else if (gamepad1.a) {
                drivetrain.driveAutoAlign(gamepad1, drivetrain.calculateAutoAlignPowerOdo(-drivetrain.calculateOdoGoalBearing(follower.getPose(), PoseManager.currentGoalPose)));
                drivetrain.holdPose = follower.getPose();
            }
            else if (gamepad1.b){
//                follower.turnTo(drivetrain.calculateOdoGoalAngle(follower.getPose(), drivetrain.BLUE_GOAL_POSITION));
            }
            else if (gamepad1.x) {
                drivetrain.resetIMU();
            }
            else {
                drivetrain.grounded = false;
                drivetrain.holdPose = follower.getPose(); //update hold pose when not grounded

                drivetrain.drive(gamepad1);
            }

//            pedropathing holdpoint control
            if (drivetrain.grounded && !drivetrain.isHoldingPose) {
                follower.holdPoint(drivetrain.holdPose);
                drivetrain.isHoldingPose = true;
            }
            else if (!drivetrain.grounded && drivetrain.isHoldingPose){
                follower.breakFollowing();
                drivetrain.isHoldingPose = false;
            }

            //mechanism intake control
            if(gamepad2.a) {
                intake.turnOnIntake();
                if (!shooter.on) {
                    intake.setDriverPower(Intake.DRIVER_INTAKE_POWER);
                    shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                }
//                else{ todo: implement this aswell
//                    intake.setDriverPower(intake.calculateDriverPower(limelight.getRange()));
//                }
            }
            else if (gamepad2.b){
                intake.turnOnOuttake();
            }
            else if (gamepad2.right_bumper){
                shooter.cycling = true;
//                shooter.setPitchPosition(Shooter.PITCH_CYCLE_POSITION); //no more cycle, just a gate override
                shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);
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
                    shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
//                    shooter.setRampPosition(0);
                }
            }
            //mechanism intake flicker control
//            if(gamepad2.dpad_left) {
//                intake.toggleFlicker();
//                intake.resetFlickerOpenTimer();
//            }

//            intake.updateFlickerOpenTimer(); //update the timer with the current time
//            intake.checkFlickerOpenTimer(gamepad2); //automatically open flicker after a second if its closed, unless the gamepad button is held down

            //mechanism shooter control
            if (gamepad2.x) {
                intake.setDriverPower(Intake.DRIVER_CLOSE_SHOOTING_POWER);
                shooter.toggleShooterClose();
            }
            else if (gamepad2.y){
                intake.setDriverPower(Intake.DRIVER_FAR_SHOOTING_POWER);
                shooter.toggleShooterFar();
            }

            //constantly update shooter velocity for close regression
            if(shooter.on){
                shooter.updateShooterVelocity();
            }

            //open the gate when scoring so balls can pass to shooter motors
            shooter.update();
            shooter.controlShooterGate();

            //DYNAMIC FAR AND CLOSE
            if (limelight.isFar) {
//                camera.setHeadingOffset(Camera.HEADING_OFFSET_FAR);
                shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_FAR);
            }
            else {
//                camera.setHeadingOffset(Camera.HEADING_OFFSET_CLOSE);
                //update regression only for close
                shooter.setCurrentTargetRPMTicksPerSecond(shooter.calculateShooterVelocityRPMLimelight(limelight.getRange()));
                shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_CLOSE);
            }

            limelight.updateIsFar();

            //indicator light control
            if (limelight.getBearing() >= -Limelight.HEADING_VALID_RANGE && limelight.getBearing() <= Limelight.HEADING_VALID_RANGE && limelight.getBearing() != 0.0){
                indicatorLight.setIndicatorLight(IndicatorLight.INDICATOR_LIGHT_GREEN);
            }
            else {
                indicatorLight.setIndicatorLight(IndicatorLight.INDICATOR_LIGHT_RED);
            }

//            telemetry.addData("intake current time:", intake.currentTime);
            telemetry.addData("shooter target velocity: ", shooter.calculateShooterVelocityRPMLimelight(limelight.getRange()));
            telemetry.addData("shooter left velocity:", shooter.shooterLeftGetVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
            telemetry.addData("shooter right velocity:", shooter.shooterRightGetVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
            telemetry.addData("shooter at velocity:", shooter.shooterAtTargetVelocity());
            telemetry.addData("balls shot:", shooter.ballsShot);


//            telemetry.addData("shooter at velocity:", shooter.shooterAtTargetVelocity());
            telemetry.addData("apriltag range:", limelight.getRange());
            telemetry.addData("apriltag bearing:", limelight.getBearing());
            telemetry.addData("drive power:", drivetrain.calculateAutoAlignPowerLimelight(limelight.getBearing()));

            telemetry.addData("pitch up time:", shooter.currentShooterClosedTime);
            telemetry.addData("pitch down time:", shooter.currentShooterOpenTime);
            telemetry.addData("pitch up debounce:", shooter.shooterClosedTimerOver());
            telemetry.addData("pitch down debounce:", shooter.shooterOpenPitchTimerOver());

            //shooter at velocity time
//            telemetry.addData("shooter at velocity time:", shooter.atVelocityTime);

            //regression zero debounce
            telemetry.addData("current regression zero time:", shooter.regressionDebounceTimer.time());
            telemetry.addData("last regression value:", shooter.lastRegressionValue);

            //grounded
            telemetry.addData("grounded:", drivetrain.grounded);

            //driver power
            telemetry.addData("driver power: ", intake.driverPower);

            telemetry.addData("current robot pose:", follower.getPose());
            telemetry.addData("camera robot pose:", limelight.getRobotPose());
            telemetry.addData("current robot odo angle:", drivetrain.calculateOdoGoalBearing(follower.getPose(), PoseManager.currentGoalPose));

            telemetry.addData("current inches from goal:", drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose));

            Drawing.drawDebug(follower);

            telemetry.update();
        }
    }
}


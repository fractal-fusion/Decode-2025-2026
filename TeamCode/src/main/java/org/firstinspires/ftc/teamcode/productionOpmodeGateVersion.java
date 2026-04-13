package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="productionOpmodeGateVersion", group="Robot")
public class productionOpmodeGateVersion extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        //build paths for teleop in init
        PathChain goToLeverPositionFromFar = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, PoseManager.currentLeverPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, PoseManager.currentLeverPose.getHeading(), 0.8))
                .build();

        PathChain goToLeverPositionFromClose = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(follower::getPose, PoseManager.currentLeverControlPoint, PoseManager.currentLeverPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, PoseManager.currentLeverPose.getHeading(), 0.8))
                .build();

        PathChain goToFarPosition = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, PoseManager.currentFarPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, PoseManager.currentFarPose.getHeading(), 0.8))
                .build();


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
            //update the gamepad2 states of the drivetrain object for the rising edge detector to work
//            drivetrain.updateGamepad(gamepad1);
            //update the gamepad2 states of the shooter object for the rising edge detector to work
            shooter.updateGamepad(gamepad2);
            //update the gamepad2 states of the intake object for the rising edge detector to work
            intake.updateGamepad(gamepad2);

            //automatic relocalization
            if (limelight.isValidResult() && drivetrain.isSlowForRelocalization(follower) && !drivetrain.isFarOdometry(follower.getPose())){

                if (limelight.isReadyToRelocalize()){
                    if (!new Pose().roughlyEquals(limelight.getFilteredPose(follower), 1)){ //recalibrate pose using limelight
                        follower.setPose(limelight.getFilteredPose(follower));
                        limelight.resetRelocalizationTimer();
                    }
                }
                else {
                    limelight.updateFilteredPoseSamples();
                }
            }
            else {
                limelight.clearSamplePoses();
            }

            //drivetrain controls (field centric drive + autoalignment)
            if (gamepad1.right_bumper){
                if (!drivetrain.isFollowing){
                    follower.holdPoint(drivetrain.holdPose);
                    drivetrain.isFollowing = true;
                }
            }
            else if (gamepad1.dpad_down){
                if(!drivetrain.isFollowing){
                    follower.followPath(goToFarPosition, true);
                    drivetrain.isFollowing = true;
                }
            }
            else if (gamepad1.dpad_right){
                if(!drivetrain.isFollowing){
                    if(drivetrain.isFarOdometry(follower.getPose())){
                        follower.followPath(goToLeverPositionFromFar, true);

                    }
                    else{
                        follower.followPath(goToLeverPositionFromClose, true);
                    }
                    drivetrain.isFollowing = true;
                }
            }
            else if (gamepad1.b) {
                drivetrain.driveAutoAlign(gamepad1, drivetrain.calculateAutoAlignPowerLimelight(limelight.getBearing()));
                drivetrain.holdPose = follower.getPose();

                if (!new Pose().roughlyEquals(limelight.getSamplePose(), 1) && !limelight.isFar){ //recalibrate pose using limelight
                    follower.setPose(limelight.getSamplePose());
                }
            }
            else if (gamepad1.y) {
                //sotm
                drivetrain.driveAutoAlign(gamepad1, drivetrain.calculatePrimaryPIDAutoAlignPowerOdo(-drivetrain.calculateOdoGoalBearing(follower.getPose(), drivetrain.calculateVirtualGoalPose(follower, drivetrain.calculateAirTime(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose)), PoseManager.currentGoalAutoAlignPose))));
//                drivetrain.driveAutoAlign(gamepad1, drivetrain.calculNGateAutoAlignPowerOdo(-drivetrain.calculateOdoGoalBearing(follower.getPose(), PoseManager.currentGoalAutoAlignPose)));
                drivetrain.holdPose = follower.getPose();
            }
            else if (gamepad1.a) {
                //pedro auto align (BAD)
//                if(!drivetrain.isFollowing){
//                    follower.turnTo(drivetrain.calculateOdoGoalAngle(follower.getPose(), PoseManager.currentGoalAutoAlignPose));
//                    drivetrain.isFollowing = true;
//                }

                //proportional drive auto align
                drivetrain.driveAutoAlign(gamepad1, drivetrain.calculateSecondaryPIDAutoAlignPowerOdo(-drivetrain.calculateOdoGoalBearing(follower.getPose(), PoseManager.currentGoalAutoAlignPose)));

                drivetrain.holdPose = follower.getPose();
            }
            else if (gamepad1.x) {
                drivetrain.resetIMU();
            }
            else {
                drivetrain.holdPose = follower.getPose(); //update hold pose when not grounded

                if (drivetrain.isFollowing){
                    follower.breakFollowing();
                    drivetrain.isFollowing = false;
                }

                drivetrain.drive(gamepad1);
            }
            //toggle tilt
//            drivetrain.toggleTilt(gamepad1);

            //mechanism intake control
            if(gamepad2.a) {
                intake.turnOnIntake();
                if (!shooter.on) {
                    intake.setDriverPower(Intake.DRIVER_INTAKE_POWER);
                    shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                }
//                else{
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
                    shooter.turnOffShooterRestingRpm();
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
                shooter.toggleShooterClose();
            }
            else if (gamepad2.y){
                intake.setDriverPower(Intake.DRIVER_FAR_SHOOTING_POWER);
                shooter.toggleShooterFar();
            }

            //constantly update shooter velocity for close regression
            //DYNAMIC FAR AND CLOSE
            if(shooter.on){
                //reduce driver speed if too close or too far to goal inside close zone
                if(drivetrain.ifSlowDriverOdometry(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose))) {
                    intake.setDriverPower(Intake.DRIVER_CLOSE_SLOW_SHOOTING_POWER);
                }
                else {
                    intake.setDriverPower(Intake.DRIVER_CLOSE_SHOOTING_POWER);
                }

                shooter.updateShooterVelocity();
                if (!drivetrain.isFarOdometry(follower.getPose())){ //only update ramp regression when close and shooter is on
                    //sotm
                    shooter.setCurrentTargetRPMTicksPerSecond(shooter.calculateShooterVelocityRPMOdometryClose(drivetrain.calculateOdoGoalDistance(follower.getPose(), drivetrain.calculateVirtualGoalPose(follower, drivetrain.calculateAirTime(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose)), PoseManager.currentGoalPose))));
                    shooter.setRampPosition(shooter.calculateRampPositionOdometry(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose)));
                    shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_CLOSE);
                }
                else{
                    shooter.setCurrentTargetRPMTicksPerSecond(shooter.calculateShooterVelocityRPMOdometryClose(drivetrain.calculateOdoGoalDistance(follower.getPose(), drivetrain.calculateVirtualGoalPose(follower, drivetrain.calculateAirTime(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose)), PoseManager.currentGoalPose))));
                    shooter.setRampPosition(shooter.calculateRampPositionOdometry(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose)));
                    shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_FAR);
                }
            }

            //open the gate when scoring so balls can pass to shooter motors
            shooter.update();
            shooter.controlShooterGate();

//            //no longer use limelight for far and close detection
//            if (limelight.isFar) {
//                camera.setHeadingOffset(Camera.HEADING_OFFSET_FAR);
//                shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_FAR);
//            }
//            else {
//                camera.setHeadingOffset(Camera.HEADING_OFFSET_CLOSE);
//                shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_CLOSE);
//            }
//            limelight.updateIsFar();

            //indicator light control
            if (gamepad1.y){
                indicatorLight.setIndicatorLight(IndicatorLight.INDICATOR_LIGHT_ORANGE);
            }
            else if (limelight.getBearing() >= -Limelight.HEADING_VALID_RANGE && limelight.getBearing() <= Limelight.HEADING_VALID_RANGE && limelight.getBearing() != 0.0){
                indicatorLight.setIndicatorLight(IndicatorLight.INDICATOR_LIGHT_GREEN);
            }
            else {
                indicatorLight.setIndicatorLight(IndicatorLight.INDICATOR_LIGHT_PURPLE);
            }

//            telemetry.addData("intake current time:", intake.currentTime);
            telemetry.addData("shooter target velocity: ", shooter.currentTargetRPMTicksPerSecond * Shooter.RPM_TO_TICKS_PER_SECOND);
            telemetry.addData("shooter left velocity:", shooter.shooterLeftGetVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
            telemetry.addData("shooter right velocity:", shooter.shooterRightGetVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
            telemetry.addData("shooter at velocity:", shooter.shooterAtTargetVelocity());
            telemetry.addData("balls shot:", shooter.ballsShot);
            telemetry.addLine("------------------------------------------------------");

//            telemetry.addData("shooter at velocity:", shooter.shooterAtTargetVelocity());
            telemetry.addData("apriltag range:", limelight.getRange());
            telemetry.addData("apriltag bearing:", limelight.getBearing());
            telemetry.addData("drive power:", drivetrain.calculateAutoAlignPowerLimelight(limelight.getBearing()));
            telemetry.addLine("------------------------------------------------------");

            telemetry.addData("pitch up time:", shooter.currentShooterClosedTime);
            telemetry.addData("pitch down time:", shooter.currentShooterOpenTime);
            telemetry.addData("pitch up debounce:", shooter.shooterClosedTimerOver());
            telemetry.addData("pitch down debounce:", shooter.shooterOpenPitchTimerOver());
            telemetry.addLine("------------------------------------------------------");

            //shooter at velocity time
//            telemetry.addData("shooter at velocity time:", shooter.atVelocityTime);

            //regression zero debounce
            telemetry.addData("current regression zero time:", shooter.regressionDebounceTimer.time());
            telemetry.addData("last regression value:", shooter.lastRegressionValue);
            telemetry.addLine("------------------------------------------------------");

            //grounded
            telemetry.addData("grounded:", drivetrain.grounded);
            telemetry.addLine("------------------------------------------------------");

            //driver power
            telemetry.addData("driver power: ", intake.driverPower);
            telemetry.addLine("------------------------------------------------------");

            telemetry.addData("current robot pose:", follower.getPose());
            telemetry.addData("camera robot pose:", limelight.getSamplePose());
            telemetry.addData("current robot odo bearing:", drivetrain.calculateOdoGoalBearing(follower.getPose(), PoseManager.currentGoalPose));
            telemetry.addData("current inches from goal:", drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose));
            telemetry.addLine("------------------------------------------------------");

            telemetry.addData("current ramp position:", shooter.calculateRampPositionOdometry(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose)));
            telemetry.addLine("------------------------------------------------------");

            telemetry.addData("drivetrain velocity", follower.getVelocity().getMagnitude());
            telemetry.addData("drivetrain is far:", drivetrain.isFarOdometry(follower.getPose()));
            telemetry.addData("current target rpm close:", shooter.calculateShooterVelocityRPMOdometryClose(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose)));
            telemetry.addData("current target rpm far:", shooter.calculateShooterVelocityRPMOdometryFar(drivetrain.calculateOdoGoalDistance(follower.getPose(), PoseManager.currentGoalPose)));
            telemetry.addLine("------------------------------------------------------");

            telemetry.addData("current relocalization timer", limelight.relocalizationTimer.getElapsedTimeSeconds());
            telemetry.addData("can relocalize", limelight.isReadyToRelocalize());
            telemetry.addData("current samples", limelight.SamplePoses.size());
            telemetry.addLine("------------------------------------------------------");

            telemetry.addData("heading PID timer", drivetrain.headingPIDTimer.getElapsedTimeSeconds());
            Drawing.drawDebug(follower);

            telemetry.update();
        }
    }
}


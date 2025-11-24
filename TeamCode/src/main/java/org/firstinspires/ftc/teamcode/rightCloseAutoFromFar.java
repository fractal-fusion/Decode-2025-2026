package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

    @Config
    @Autonomous(name="Red Close Auto From Far", group="Robot")
    @SuppressWarnings("FieldCanBeLocal")
    public class rightCloseAutoFromFar extends LinearOpMode {
        Drivetrain drivetrain;
        Shooter shooter;
        Intake intake;
//        Camera camera;
        private Follower follower; //initialize the follower object
        private Timer pathTimer, opmodeTimer; //declare the time variables used when checking for path completion
        private Pose currentPose;
        private int pathState = 0; //finite state machine variable
        private boolean init = true;
        public static double INTAKE_DELAY_TIME = 0.5;
        public static double RELEASE_BALLS_WAIT_TIME = 0.15; //time to wait at the chamber
        public static double SCORE_HEADING_OFFSET = -5; //score heading offset since center of goals are not exactly 45 degrees

        //variables to keep track of how long each score took in order to implement failsafes based on the opmode timer
        private double scorePreloadTime = 0.0;
        private double scorePickupTopTime = 0.0;
        private double scorePickupMiddleTime = 0.0;
        private double scorePickupBottomTime = 0.0;

        public double scoreHeading = Math.toRadians(45 + SCORE_HEADING_OFFSET);

        private PathChain scorePreload, grabPickupBottom, scorePickupBottom, grabPickupMiddle, scorePickupMiddle, grabPickupTop, scorePickupTop, goToReleaseBalls, goToPark; //define path chains (muliple paths interpolated)

        private final Pose startPose = new Pose(89.5, 8, Math.toRadians(90)); // Start Pose of our robot
        private final Pose scorePose = new Pose(98, 100, scoreHeading);
        private final Pose grabPickupTopPose = new Pose(128, 84, Math.toRadians(0));
        private final Pose grabPickupTopPoseControlPoint1 = new Pose(42.978, 82.633);
        private final Pose releaseBallsPose = new Pose(130, 69, Math.toRadians(0));
        private final Pose releaseBallsPoseControlPoint1 = new Pose(94.818, 69.784);
        private final Pose grabPickupMiddlePose = new Pose(132, 60, Math.toRadians(0));
        private final Pose grabPickupMiddlePoseControlPoint1 = new Pose(33.452, 55.606);
        private final Pose scorePickupMiddlePoseControlPoint1= new Pose(102.793, 69.341);
        private final Pose grabPickupBottomPose = new Pose(132, 36, Math.toRadians(0));
        private final Pose grabPickupBottomPoseControlPoint1 = new Pose(27.470, 28.356);
        private final Pose parkPose = new Pose(86,110, Math.toRadians(220));

        public void buildPaths() {
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();
            grabPickupTop = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, grabPickupTopPoseControlPoint1, grabPickupTopPose))
//                    .addPath(new BezierLine(scorePose, grabPickupTopPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), grabPickupTopPose.getHeading())
                    .addPoseCallback(new Pose(124, 84), intake::holdFlicker, 0.5)
                    .build();
            goToReleaseBalls = follower.pathBuilder()
                    .addPath(new BezierCurve(grabPickupTopPose, releaseBallsPoseControlPoint1, releaseBallsPose))
                    .setLinearHeadingInterpolation(grabPickupTopPose.getHeading(), releaseBallsPose.getHeading())
                    .build();
            scorePickupTop = follower.pathBuilder()
                    .addPath(new BezierLine(grabPickupTopPose, scorePose))
                    .setLinearHeadingInterpolation(grabPickupTopPose.getHeading(), scorePose.getHeading())
                    .build();
            grabPickupMiddle = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, grabPickupMiddlePoseControlPoint1, grabPickupMiddlePose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), grabPickupMiddlePose.getHeading())
                    .addPoseCallback(new Pose(126, 58), intake::holdFlicker, 0.5)
                    .build();
            scorePickupMiddle = follower.pathBuilder()
                    .addPath(new BezierCurve(grabPickupMiddlePose, scorePickupMiddlePoseControlPoint1, scorePose))
                    .setLinearHeadingInterpolation(grabPickupMiddlePose.getHeading(), scorePose.getHeading())
                    .build();
            grabPickupBottom = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, grabPickupBottomPoseControlPoint1, grabPickupBottomPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), grabPickupBottomPose.getHeading())
                    .addPoseCallback(new Pose(126, 36), intake::holdFlicker, 0.5)
                    .build();
            scorePickupBottom = follower.pathBuilder()
                    .addPath(new BezierLine(grabPickupBottomPose, scorePose))
                    .setLinearHeadingInterpolation(grabPickupBottomPose.getHeading(), scorePose.getHeading())
                    .build();
            goToPark = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, parkPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                    .build();
        }


        @Override
        public void runOpMode() throws InterruptedException {
            //initialize subsystems
            drivetrain = new Drivetrain(this);
            shooter = new Shooter(this);
            intake = new Intake(this, Intake.FLICKER_CLOSE_POSITION);
//            camera = new Camera(this, 3);

            //initialize timers so they can be checked in the state machine
            pathTimer = new Timer();
            opmodeTimer = new Timer();

            follower = Constants.createFollower(hardwareMap); //create pedropathing follower
            follower.setStartingPose(startPose);
            follower.setMaxPower(0.8); //decrease max power to prevent overshoot

            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION); //set gate to closed position on initialize

            buildPaths(); //build all paths

            if (isStopRequested()) return;

            waitForStart();

            opmodeTimer.resetTimer(); //reset opmode total timer on start

            setPathState(0);

            while (opModeIsActive()) {
                follower.update(); //update follower
                currentPose = follower.getPose(); //update current pose

                updateStateMachine();

                shooter.update();
                shooter.controlShooterGate();

                telemetry.addData("shooter left velocity:", shooter.shooterLeftGetVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
                telemetry.addData("shooter right velocity:", shooter.shooterRightGetVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
                telemetry.addData("shooter at velocity:", shooter.shooterAtTargetVelocity());
                telemetry.addData("balls shot:", shooter.ballsShot);
                telemetry.addData("pitch closed time:", shooter.currentShooterClosedTime);
                telemetry.addData("pitch open time:", shooter.currentShooterOpenTime);
                telemetry.addData("shooter closed on cooldown:", shooter.shooterClosedTimerOver());
                telemetry.addData("shooter open on cooldown:", shooter.shooterOpenPitchTimerOver());
                telemetry.addData("score preloads time: ", scorePreloadTime);
                telemetry.addData("score top time: ", scorePickupTopTime);
                telemetry.addData("score middle time: ", scorePickupMiddleTime);
                telemetry.addData("score bottom time: ", scorePickupBottomTime);


                telemetry.update();

            }
        }

        public void updateStateMachine() {
            switch (pathState) {
                case 0: //move to score position for preload
                    //hold the flicker in
                    if (init){
                        intake.setFlickerPosition(Intake.FLICKER_HOLD_POSITION);
                        init = false;
                    }
                    else{ //move to scoring position
                        follower.followPath(scorePreload, true);
                        intializeBurstClose(); //prestart shooter
                        turnOnShooterAuto();
                        setPathState(1);
                    }
                    break;
                case 1: //score preloads
                    if (!follower.isBusy()) {
                        if(init){
//                            intializeBurstClose();
//                            turnOnShooterAuto();
                            shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                            init = false;
                        }
                        else{
                            if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                                intake.turnOnIntake();
                                intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);
                            }

                            if (shooter.ballsShot >= 3 || opmodeTimer.getElapsedTimeSeconds() > 4) {
                                scorePreloadTime = opmodeTimer.getElapsedTimeSeconds();

                                shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                                turnOffShooterAuto();
                                setPathState(2); //end
                            }
                        }
                    }
                    break;
                case 2: // intake top row
                    if (!follower.isBusy()) {
                        if (init){
                            intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);
                            init = false;
                        }
                        else{
                            follower.followPath(grabPickupTop, true);
                            setPathState(3);
                        }
                    }
                    break;
                case 3: //release preload balls
                    if (!follower.isBusy()){
                        if (init){
                            intake.turnOffIntake();
                            init = false;
                        }
                        else{
                            follower.followPath(goToReleaseBalls);
                            if (pathTimer.getElapsedTimeSeconds() > RELEASE_BALLS_WAIT_TIME) {
                                setPathState(4);
                            }
                        }
                    }
                    break;
                case 4: //move to score position for top row
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickupTop, true);
                        intializeBurstClose(); //prestart shooter
                        turnOnShooterAuto();
                        setPathState(5);
                    }
                    break;
                case 5: //score top row
                    if (!follower.isBusy()) {
                        if(init){
//                            intializeBurstClose();
//                            turnOnShooterAuto();
                            shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                            init = false;
                        }
                        else{
                            if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                                intake.turnOnIntake();
                                intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);
                            }

                            if (shooter.ballsShot >= 6 || opmodeTimer.getElapsedTimeSeconds() > 13) {
                                scorePickupTopTime = opmodeTimer.getElapsedTimeSeconds();

                                shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                                turnOffShooterAuto();
                                setPathState(6);
                            }
                        }
                    }
                    break;
                case 6: // intake middle row
                    if (!follower.isBusy()) {
                        if (init){
                            intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);
                            init = false;
                        }
                        else{
                            follower.followPath(grabPickupMiddle, true);
                            setPathState(7);
                        }
                    }
                    break;
                case 7: //move to score position for middle row
                    if (!follower.isBusy()) {
                        if (init){
                            intake.turnOffIntake();
                            init = false;
                        }
                        else{
                            follower.followPath(scorePickupMiddle, true);
                            intializeBurstClose(); //prestart shooter
                            turnOnShooterAuto();
                            setPathState(8);
                        }
                    }
                    break;
                case 8: //score middle row
                    if (!follower.isBusy()) {
                        if(init){
//                            intializeBurstClose();
//                            turnOnShooterAuto();
                            shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                            init = false;
                        }
                        else{
                            if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                                intake.turnOnIntake();
                                intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);
                            }

                            if (shooter.ballsShot >= 9 || opmodeTimer.getElapsedTimeSeconds() > 19) {
                                scorePickupMiddleTime = opmodeTimer.getElapsedTimeSeconds();

                                shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                                turnOffShooterAuto();
                                setPathState(9);
                            }
                        }
                    }
                    break;
                case 9: // intake bottom row
                    if (!follower.isBusy()) {
                        if (init){
                            intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);
                            init = false;
                        }
                        else{
                            follower.followPath(grabPickupBottom, true);
                            setPathState(10);
                        }
                    }
                    break;
                case 10: //move to score position for bottom row
                    if (!follower.isBusy()) {
                        if (init){
                            intake.turnOffIntake();
                            init = false;
                        }
                        else{
                            follower.followPath(scorePickupBottom, true);
                            intializeBurstClose(); //prestart shooter
                            turnOnShooterAuto();
                            setPathState(11);
                        }
                    }
                    break;
                case 11: //score bottom row
                    if (!follower.isBusy()) {
                        if(init){
//                            intializeBurstClose();
//                            turnOnShooterAuto();
                            shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                            init = false;
                        }
                        else{
                            if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                                intake.turnOnIntake();
                                intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);
                            }

                            if (shooter.ballsShot >= 12 || opmodeTimer.getElapsedTimeSeconds() > 26) {
                                scorePickupBottomTime = opmodeTimer.getElapsedTimeSeconds();

                                shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                                turnOffShooterAuto();
                                intake.turnOffIntake();
                                setPathState(12);
                            }
                        }
                    }
                    break;
                case 12:
                    if(!follower.isBusy()){
                        follower.followPath(goToPark);
                        setPathState(-1);
                    }
                    break;
            } //run state machine
        }

        public void intializeBurstClose(){
            shooter.setCurrentShooterClosedSeconds(Shooter.CLOSE_DEBOUNCE);
            shooter.setCurrentTargetRPMTicksPerSecond(Shooter.CLOSE_AUTO_TARGET_RPM);
            shooter.setRampPosition(Shooter.CLOSE_RAMP_SCORE_POSITION);
            shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_CLOSE);
        }

        public void initalizeBurstFar(){
            shooter.setCurrentShooterClosedSeconds(Shooter.FAR_DEBOUNCE);
            shooter.setCurrentTargetRPMTicksPerSecond(Shooter.FAR_TARGET_RPM);
            shooter.setRampPosition(Shooter.FAR_RAMP_SCORE_POSITION);
            shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_FAR);
        }

        public void turnOnShooterAuto(){
            shooter.turnOnShooter();
            shooter.on = true;
        }

        public void turnOffShooterAuto(){
            shooter.turnOffShooter();
            shooter.on = false;
        }

        public void setPathState(int pState) {
            pathState = pState;
            init = true;
            pathTimer.resetTimer();
            shooter.resetShooterClosedTimer();
            shooter.resetShooterOpenTimer();
        }

    }
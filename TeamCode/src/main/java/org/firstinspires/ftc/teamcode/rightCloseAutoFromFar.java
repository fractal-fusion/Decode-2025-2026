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
        Camera camera;
        private Follower follower; //initialize the follower object
        private Timer pathTimer, opmodeTimer; //declare the time variables used when checking for path completion
        private Pose currentPose;
        private int pathState = 0; //finite state machine variable
        private boolean init = true;
        public static double INTAKE_DELAY_TIME = 0.5;
        public static double SCORE_HEADING_OFFSET = -5; //score heading offset since center of goals are not exactly 45 degrees

        public double scoreHeading = Math.toRadians(45 + SCORE_HEADING_OFFSET);

        private PathChain scorePreload, grabPickupBottom, scorePickupBottom, grabPickupMiddle, scorePickupMiddle, grabPickupTop, scorePickupTop, goToPark; //define path chains (muliple paths interpolated)

        private final Pose startPose = new Pose(88, 8, Math.toRadians(90)); // Start Pose of our robot
        private final Pose scorePose = new Pose(86, 90, scoreHeading); //TODO: change this to not be middle
        private final Pose grabPickupTopPose = new Pose(128, 84, Math.toRadians(0));
        private final Pose grabPickupTopPoseControlPoint1 = new Pose(63.138, 78.203);
        private final Pose grabPickupMiddlePose = new Pose(130, 58, Math.toRadians(0));
        private final Pose grabPickupMiddlePoseControlPoint1 = new Pose(62.916, 56.713);
        private final Pose parkPose = new Pose(86,110, Math.toRadians(220));




        //TODO: SET OTHER POSES


        //BUILD PATHS TODO: actually build them using poses from visualizer
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
                    .addPath(new BezierLine(grabPickupMiddlePose, scorePose))
                    .setLinearHeadingInterpolation(grabPickupMiddlePose.getHeading(), scorePose.getHeading())
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
            camera = new Camera(this, 3);

            //initialize timers so they can be checked in the state machine
            pathTimer = new Timer();
            opmodeTimer = new Timer();

            follower = Constants.createFollower(hardwareMap); //create pedropathing follower
            follower.setStartingPose(startPose);
            follower.setMaxPower(0.8); //decrease max power to prevent overshoot

            shooter.setPitchPosition(Shooter.PITCH_INTAKE_POSITION); //set pitch to intake position on initialize

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
                shooter.controlShooterPitch();

                telemetry.addData("shooter left velocity:", shooter.shooterLeftGetVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
                telemetry.addData("shooter right velocity:", shooter.shooterRightGetVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
                telemetry.addData("shooter at velocity:", shooter.shooterAtTargetVelocity());
                telemetry.addData("balls shot:", shooter.ballsShot);
                telemetry.addData("pitch up time:", shooter.currentPitchUpTime);
                telemetry.addData("pitch down time:", shooter.currentPitchDownTime);
                telemetry.addData("pitch up debounce:", shooter.pitchUpDebounceTimerOver());
                telemetry.addData("pitch down debounce:", shooter.pitchDownDebounceTimerOver());
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
                        setPathState(1);
                    }
                    break;
                case 1: //score preloads
                    if (!follower.isBusy()) {
                        if(init){
                            intializeBurstClose();
                            turnOnShooterAuto();

                            init = false;
                        }
                        else{
                            if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                                intake.turnOnIntake();
                                intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);
                            }

                            if (shooter.ballsShot >= 3) {
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
                case 3: //move to score position for top row
                    if (!follower.isBusy()) {
                        if (init){
                            intake.turnOffIntake();
                            init = false;
                        }
                        else{
                            follower.followPath(scorePickupTop, true);
                            setPathState(4);
                        }
                    }
                    break;
                case 4: //score top row
                    if (!follower.isBusy()) {
                        if(init){
                            intializeBurstClose();
                            turnOnShooterAuto();

                            init = false;
                        }
                        else{
                            if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                                intake.turnOnIntake();
                                intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);
                            }

                            if (shooter.ballsShot >= 6) {
                                turnOffShooterAuto();
                                setPathState(5); //end
                            }
                        }
                    }
                    break;
                case 5: // intake top row
                    if (!follower.isBusy()) {
                        if (init){
                            intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);
                            init = false;
                        }
                        else{
                            follower.followPath(grabPickupMiddle, true);
                            setPathState(6);
                        }
                    }
                    break;
                case 6: //move to score position for top row
                    if (!follower.isBusy()) {
                        if (init){
                            intake.turnOffIntake();
                            init = false;
                        }
                        else{
                            follower.followPath(scorePickupMiddle, true);
                            setPathState(7);
                        }
                    }
                    break;
                case 7: //score top row
                    if (!follower.isBusy()) {
                        if(init){
                            intializeBurstClose();
                            turnOnShooterAuto();

                            init = false;
                        }
                        else{
                            if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                                intake.turnOnIntake();
                                intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);
                            }

                            if (shooter.ballsShot >= 9) {
                                turnOffShooterAuto();
                                intake.turnOffIntake();
                                setPathState(8); //end
                            }
                        }
                    }
                    break;
                case 8:
                    if(!follower.isBusy()){
                        follower.followPath(goToPark);
                        setPathState(-1);
                    }
                    break;
            } //run state machine
        }

        public void intializeBurstClose(){
            shooter.setCurrentTargetRPMTicksPerSecond(Shooter.CLOSE_AUTO_TARGET_RPM);
            shooter.setRampPosition(Shooter.CLOSE_RAMP_SCORE_POSITION);
            shooter.setTargetRPMToleranceRPM(Shooter.TARGET_RPM_TOLERANCE_RPM_CLOSE);
        }

        public void initalizeBurstFar(){
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
            shooter.resetPitchUpTimer();
            shooter.resetPitchDownTimer();
        }

    }
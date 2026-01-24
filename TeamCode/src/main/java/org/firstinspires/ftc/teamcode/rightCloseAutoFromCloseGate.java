package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="Red Close Auto From Close Gate", group="Robot")
@SuppressWarnings("FieldCanBeLocal")
public class rightCloseAutoFromCloseGate extends LinearOpMode {
    Drivetrain drivetrain;
    Shooter shooter;
    Intake intake;
    //    Camera camera;
    private Follower follower; //initialize the follower object
    private Timer pathTimer, opmodeTimer; //declare the time variables used when checking for path completion
    private Pose currentPose;
    private int pathState = 0; //finite state machine variable
    private boolean init = true;
    public static double INTAKE_DELAY_TIME = 0.05;
    public static double INTAKE_DELAY_TIME_PRELOAD = 0.1;
    public static double WALL_HUMAN_PLAYER_X = 128;
    public static double INTAKE_HUMAN_PLAYER_X = 136.43;
//    public static double INTAKE_HUMAN_PLAYER_FLICKER_TIME = 3;

    public static double RELEASE_BALLS_WAIT_TIME = 0.3; //time to wait at the chamber
    public static double COLLECT_BALLS_WAIT_TIME = 0.3;
    public static double HEADING_INTERPOLATION_END_PERCENTAGE = 0.65;
    public static double AUTO_Y_OFFSET = 0;
    public static double INTAKE_X_OFFSET = 0;
    public static double RELEASE_BALLS_Y = 74.2;
    public static double SCORE_HEADING_OFFSET = -5; //score heading offset since center of goals are not exactly 45 degrees
    public static double SCORE_HEADING_PRELOAD_TOLERANCE = 0.1;
    public static double SCORE_HEADING_PRELOAD = 44.5;
    public static double MAX_POWER = 1;
    public static double INTAKE_HUMAN_PLAYER_MAX_POWER = 0.45;

    //variables to keep track of how long each score took in order to implement failsafes based on the opmode timer
    private double scorePreloadTime = 0.0;
    private double scorePickupTopTime = 0.0;
    private double scorePickupMiddleTime = 0.0;
    private double scorePickupBottomTime = 0.0;
    private double scorePickupHumanPlayerTime = 0.0;
    public double scoreHeading = Math.toRadians(45 + SCORE_HEADING_OFFSET);
    public static double COLLECT_HEADING = 44.5;

    private PathChain scorePreload, grabPickupBottom, scorePickupBottom, grabPickupMiddle, scorePickupMiddle, grabPickupTop, scorePickupTop, goToWallHumanPlayer, grabPickupHumanPlayer, scorePickupHumanPlayer, goToReleaseBalls, collectBalls, moveBackCollectBalls, scoreCollectBalls, goToPark; //define path chains (muliple paths interpolated)

    private final Pose startPose = new Pose(129, 115+AUTO_Y_OFFSET, Math.toRadians(180)); // Start Pose of our robot
    private final Pose scorePose = new Pose(90, 94, scoreHeading);
    private final Pose scorePreloadPose = new Pose(90, 94, Math.toRadians(SCORE_HEADING_PRELOAD));
    private final Pose grabPickupTopPose = new Pose(127 + INTAKE_X_OFFSET, 84, Math.toRadians(0));
    private final Pose grabPickupTopPoseControlPoint1 = new Pose(92, 81);
    private final Pose releaseBallsPose = new Pose(128.5, RELEASE_BALLS_Y, Math.toRadians(0));
    private final Pose releaseBallsPoseControlPoint1 = new Pose(98.141, 66.904);
    private final Pose collectBallsPose = new Pose(135, 60, Math.toRadians(COLLECT_HEADING));
    private final Pose collectBallsPoseControlPoint1 = new Pose(105, 72);
    private final Pose moveBackCollectBallsPose = new Pose(135, 50, Math.toRadians(COLLECT_HEADING));
    private final Pose grabPickupMiddlePose = new Pose(132 + INTAKE_X_OFFSET, 59.5, Math.toRadians(0));
    private final Pose grabPickupMiddlePoseControlPoint1 = new Pose(84, 54);
    private final Pose scorePickupMiddlePoseControlPoint1 = new Pose(102.793, 69.341);
    private final Pose grabPickupBottomPose = new Pose(132.5 + INTAKE_X_OFFSET, 36, Math.toRadians(0));
    private final Pose grabPickupBottomPoseControlPoint1 = new Pose(86, 24);
    private final Pose goToWallHumanPlayerPose = new Pose(WALL_HUMAN_PLAYER_X, 45, Math.toRadians(315));
    private final Pose grabPickupHumanPlayerPose = new Pose(INTAKE_HUMAN_PLAYER_X, 5, Math.toRadians(270));
    private final Pose parkPose = new Pose(100,70, Math.toRadians(0));

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
                .setHeadingConstraint(SCORE_HEADING_PRELOAD_TOLERANCE)
                .build();
        grabPickupTop = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, grabPickupTopPoseControlPoint1, grabPickupTopPose))
//                    .addPath(new BezierLine(scorePose, grabPickupTopPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabPickupTopPose.getHeading(), HEADING_INTERPOLATION_END_PERCENTAGE)
//                .addPoseCallback(new Pose(126, 84), intake::holdFlicker, 0.5)
                .build();
        goToReleaseBalls = follower.pathBuilder()
                .addPath(new BezierCurve(grabPickupMiddlePose, releaseBallsPoseControlPoint1, releaseBallsPose))
//                .setVelocityConstraint(1)
                .setLinearHeadingInterpolation(grabPickupMiddlePose.getHeading(), releaseBallsPose.getHeading())
                .build();
        scorePickupTop = follower.pathBuilder()
                .addPath(new BezierLine(grabPickupTopPose, scorePose))
                .setLinearHeadingInterpolation(grabPickupTopPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickupMiddle = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, grabPickupMiddlePoseControlPoint1, grabPickupMiddlePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabPickupMiddlePose.getHeading(), HEADING_INTERPOLATION_END_PERCENTAGE)
//                .addPoseCallback(new Pose(130, 58), intake::holdFlicker, 0.5)
                .build();
        scorePickupMiddle = follower.pathBuilder()
                .addPath(new BezierCurve(grabPickupMiddlePose, scorePickupMiddlePoseControlPoint1, scorePose))
                .setLinearHeadingInterpolation(grabPickupMiddlePose.getHeading(), scorePose.getHeading())
                .build();
        grabPickupBottom = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, grabPickupBottomPoseControlPoint1, grabPickupBottomPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabPickupBottomPose.getHeading(), HEADING_INTERPOLATION_END_PERCENTAGE)
//                .addPoseCallback(new Pose(130, 36), intake::holdFlicker, 0.5)
                .build();
        scorePickupBottom = follower.pathBuilder()
                .addPath(new BezierLine(grabPickupBottomPose, scorePose))
                .setLinearHeadingInterpolation(grabPickupBottomPose.getHeading(), scorePose.getHeading())
                .build();
        goToWallHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goToWallHumanPlayerPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goToWallHumanPlayerPose.getHeading(), HEADING_INTERPOLATION_END_PERCENTAGE)
                .build();
        grabPickupHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(goToWallHumanPlayerPose, grabPickupHumanPlayerPose))
                .setLinearHeadingInterpolation(goToWallHumanPlayerPose.getHeading(), grabPickupHumanPlayerPose.getHeading())
                .setNoDeceleration()
//                .addTemporalCallback(INTAKE_HUMAN_PLAYER_FLICKER_TIME, intake::holdFlicker)
                .build();
        scorePickupHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(grabPickupHumanPlayerPose, scorePose))
                .setLinearHeadingInterpolation(grabPickupHumanPlayerPose.getHeading(), scorePose.getHeading(), HEADING_INTERPOLATION_END_PERCENTAGE)
                .build();
        collectBalls = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, collectBallsPoseControlPoint1, collectBallsPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), collectBallsPose.getHeading())
                .build();
        moveBackCollectBalls = follower.pathBuilder()
                .addPath(new BezierLine(collectBallsPose, moveBackCollectBallsPose))
                .setLinearHeadingInterpolation(collectBallsPose.getHeading(), moveBackCollectBallsPose.getHeading())
                .build();
        scoreCollectBalls = follower.pathBuilder()
                .addPath(new BezierLine(moveBackCollectBallsPose, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
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
//        camera = new Camera(this, 3);

        //initialize timers so they can be checked in the state machine
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap); //create pedropathing follower
        follower.setStartingPose(startPose);
        follower.setMaxPower(MAX_POWER);

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
//            shooter.controlShooterGate();

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
            telemetry.addData("score human player time: ", scorePickupHumanPlayerTime);



            telemetry.update();

        }
    }

    public void updateStateMachine() {
        switch (pathState) {
            case 0: //move to score position for preload
                //hold the flicker in
                if (init){
                    //intake.setFlickerPosition(Intake.FLICKER_HOLD_POSITION);

                    init = false;
                }
                else{ //move to scoring position
                    follower.followPath(scorePreload, true);
                    intializeBurstClosePreload(); //prestart shooter
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
                        if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME_PRELOAD) { //additional time to compensate for short path distance, not giving enough time for pid to adjust accordingly
                            intake.turnOnIntakeAuto();
                            //intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);

                        }

                        if (pathTimer.getElapsedTimeSeconds() > AutoOverrideTimes.OVERRIDE_PRELOAD_TIME) {
                            scorePreloadTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 3;
                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
//                            turnOffShooterAuto(); commented out to reduce power draw from turning shooter off and on
                            setPathState(2); //end
                        }
                    }
                }
                break;
            case 2: // intake middle row
                if (!follower.isBusy()) {
                    if (init){
                        //intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);
                        init = false;
                    }
                    else{
                        follower.followPath(grabPickupMiddle, true);
                        setPathState(3);
                    }
                }
                break;
            case 3: //move to score position for middle row
                if (!follower.isBusy()) {
                    if (init){
                        intake.turnOffIntake();
                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);
                        init = false;
                    }
                    else{
                        follower.followPath(scorePickupMiddle, true);
                        setPathState(4);
                    }
                }
                break;
            case 4: //score middle row
                if (!follower.isBusy()) {
                    if(init){
                        shooter.ballsShot = 3;
//                            intializeBurstClose();
//                            turnOnShooterAuto();
                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                        init = false;
                    }
                    else{
                        if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                            intake.turnOnIntakeAuto();
                            //intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);

                        }

                        if (pathTimer.getElapsedTimeSeconds() > AutoOverrideTimes.OVERRIDE_MIDDLE_ROW_TIME) {
                            scorePickupMiddleTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 6;
                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                            turnOffShooterAuto();
                            setPathState(5);
                        }
                    }
                }
                break;
            case 5: //collect balls from gate
                if (!follower.isBusy()){
                    if (init){
                        intake.turnOnIntakeAuto();
                        init = false;
                    }
                    else {
                        follower.followPath(collectBalls, true);
                        if (pathTimer.getElapsedTimeSeconds() > RELEASE_BALLS_WAIT_TIME) {
                            setPathState(6);
                        }
                    }
                }
            case 6: //move back to collect all balls and then to scoring position
                if (!follower.isBusy()){
                    if (init){
                        init = false;
                    }
                    else {
                        follower.followPath(moveBackCollectBalls, true);
                        if (pathTimer.getElapsedTimeSeconds() > COLLECT_BALLS_WAIT_TIME) {
                            setPathState(7);
                        }
                    }
                }
            case 7: //move to score position for collected balls
                if (!follower.isBusy()) {
                    if (init){
                        intake.turnOffIntake();
                        intializeBurstClosePreload(); //prestart shooter
                        turnOnShooterAuto();
                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);
                        init = false;
                    }
                    else{
                        follower.followPath(scoreCollectBalls, true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    if(init){
                        shooter.ballsShot = 6;
//                            intializeBurstClose();
//                            turnOnShooterAuto();
                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                        init = false;
                    }
                    else{
                        if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME) {
                            intake.turnOnIntakeAuto();
                            //intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);

                        }

                        if (pathTimer.getElapsedTimeSeconds() > AutoOverrideTimes.OVERRIDE_COLLECT_BALLS_TIME) {
                            scorePickupMiddleTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 9;
                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                            turnOffShooterAuto();
                            setPathState(9);
                        }
                    }
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    follower.followPath(goToPark);
                    setPathState(-1);
//                    if (init){
//                        intake.turnOffIntake();
//                        init = false;
//                    }
//                    else{
//                        follower.followPath(goToPark);
//                        setPathState(-1);
//                    }
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

    public void intializeBurstClosePreload(){
        shooter.setCurrentShooterClosedSeconds(Shooter.CLOSE_DEBOUNCE);
        shooter.setCurrentTargetRPMTicksPerSecond(Shooter.CLOSE_AUTO_TARGET_RPM_PRELOAD);
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
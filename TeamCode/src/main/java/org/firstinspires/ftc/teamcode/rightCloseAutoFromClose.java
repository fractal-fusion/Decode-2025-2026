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
@Autonomous(name="Red Close Auto From Close", group="Robot")
@SuppressWarnings("FieldCanBeLocal")
public class rightCloseAutoFromClose extends LinearOpMode {
    Drivetrain drivetrain;
    Shooter shooter;
    Intake intake;
    //    Camera camera;
    private Follower follower; //initialize the follower object
    private Timer pathTimer, opmodeTimer; //declare the time variables used when checking for path completion
    private Pose currentPose;
    private int pathState = 0; //finite state machine variable
    private boolean init = true;
    public static double INTAKE_DELAY_TIME = 0.5;
    public static double INTAKE_DELAY_TIME_PRELOAD = 2;
//    public static double INTAKE_HUMAN_PLAYER_X = 136.75;
//    public static double INTAKE_HUMAN_PLAYER_FLICKER_TIME = 3;

    public static double RELEASE_BALLS_WAIT_TIME = 0.05; //time to wait at the chamber
    public static double HEADING_INTERPOLATION_END_PERCENTAGE = 0.65;
    public static double AUTO_Y_OFFSET = 0;
    public static double INTAKE_X_OFFSET = 0;
    public static double RELEASE_BALLS_Y = 73.5;
    public static double SCORE_HEADING_OFFSET = -5; //score heading offset since center of goals are not exactly 45 degrees
    public static double MAX_POWER = 0.9;

    //variables to keep track of how long each score took in order to implement failsafes based on the opmode timer
    private double scorePreloadTime = 0.0;
    private double scorePickupTopTime = 0.0;
    private double scorePickupMiddleTime = 0.0;
    private double scorePickupBottomTime = 0.0;
    private double scorePickupHumanPlayerTime = 0.0;
    public static double OVERRIDE_PRELOAD_TIME = 4;
    public static double OVERRIDE_TOP_ROW_TIME = 13;
    public static double OVERRIDE_MIDDLE_ROW_TIME = 22;
    public static double OVERRIDE_BOTTOM_ROW_TIME = 28;

    public double scoreHeading = Math.toRadians(45 + SCORE_HEADING_OFFSET);

    private PathChain scorePreload, grabPickupBottom, scorePickupBottom, grabPickupMiddle, scorePickupMiddle, grabPickupTop, scorePickupTop, goToWallHumanPlayer, grabPickupHumanPlayer, scorePickupHumanPlayer, goToReleaseBalls, goToPark; //define path chains (muliple paths interpolated)

    private final Pose startPose = new Pose(129, 115+AUTO_Y_OFFSET, Math.toRadians(180)); // Start Pose of our robot
    private final Pose scorePose = new Pose(98, 100, scoreHeading);
    private final Pose grabPickupTopPose = new Pose(128 + INTAKE_X_OFFSET, 84, Math.toRadians(0));
    private final Pose grabPickupTopPoseControlPoint1 = new Pose(59.593, 79.089);
    private final Pose releaseBallsPose = new Pose(128.5, RELEASE_BALLS_Y, Math.toRadians(0));
    private final Pose releaseBallsPoseControlPoint1 = new Pose(117.858, 75.323);
    private final Pose grabPickupMiddlePose = new Pose(132 + INTAKE_X_OFFSET, 60, Math.toRadians(0));
    private final Pose grabPickupMiddlePoseControlPoint1 = new Pose(55.606, 51.175);
    private final Pose scorePickupMiddlePoseControlPoint1 = new Pose(102.793, 69.341);
    private final Pose grabPickupBottomPose = new Pose(132.5 + INTAKE_X_OFFSET, 36, Math.toRadians(0));
    private final Pose grabPickupBottomPoseControlPoint1 = new Pose(60.48, 24.812);
//    private final Pose goToWallHumanPlayerPose = new Pose(136, 45, Math.toRadians(270));
//    private final Pose grabPickupHumanPlayerPose = new Pose(INTAKE_HUMAN_PLAYER_X, 12, Math.toRadians(270));
    private final Pose parkPose = new Pose(100,70, Math.toRadians(0));

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickupTop = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, grabPickupTopPoseControlPoint1, grabPickupTopPose))
//                    .addPath(new BezierLine(scorePose, grabPickupTopPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabPickupTopPose.getHeading(), HEADING_INTERPOLATION_END_PERCENTAGE)
//                .addPoseCallback(new Pose(126, 84), intake::holdFlicker, 0.5)
                .build();
        goToReleaseBalls = follower.pathBuilder()
                .addPath(new BezierCurve(grabPickupTopPose, releaseBallsPoseControlPoint1, releaseBallsPose))
//                .setVelocityConstraint(1)
                .setLinearHeadingInterpolation(grabPickupTopPose.getHeading(), releaseBallsPose.getHeading())
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
//        goToWallHumanPlayer = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, goToWallHumanPlayerPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), goToWallHumanPlayerPose.getHeading(), HEADING_INTERPOLATION_END_PERCENTAGE)
//                .setNoDeceleration()
//                .build();
//        grabPickupHumanPlayer = follower.pathBuilder()
//                .addPath(new BezierLine(goToWallHumanPlayerPose, grabPickupHumanPlayerPose))
//                .setTangentHeadingInterpolation()
//                .addTemporalCallback(INTAKE_HUMAN_PLAYER_FLICKER_TIME, intake::holdFlicker)
//                .build();
//        scorePickupHumanPlayer = follower.pathBuilder()
//                .addPath(new BezierLine(grabPickupHumanPlayerPose, scorePose))
//                .setLinearHeadingInterpolation(grabPickupHumanPlayerPose.getHeading(), scorePose.getHeading(), HEADING_INTERPOLATION_END_PERCENTAGE)
//                .build();
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
                        if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME_PRELOAD) { //additional time to compensate for short path distance, not giving enough time for pid to adjust accordingly
                            intake.turnOnIntakeAuto();
                            //intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);

                        }

                        if (shooter.ballsShot >= 3 || opmodeTimer.getElapsedTimeSeconds() > OVERRIDE_PRELOAD_TIME) {
                            scorePreloadTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 3;
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
                        //intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);

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

                        if (shooter.ballsShot >= 6 || opmodeTimer.getElapsedTimeSeconds() > OVERRIDE_TOP_ROW_TIME) {
                            scorePickupTopTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 6;
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
                        //intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);

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

                        if (shooter.ballsShot >= 9 || opmodeTimer.getElapsedTimeSeconds() > OVERRIDE_MIDDLE_ROW_TIME) {
                            scorePickupMiddleTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 9;
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
                        //intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);

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
                        shooter.ballsShot = 9;

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

                        if (shooter.ballsShot >= 12 || opmodeTimer.getElapsedTimeSeconds() > OVERRIDE_BOTTOM_ROW_TIME) {
                            scorePickupBottomTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 12;
                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                            turnOffShooterAuto();
                            intake.turnOffIntake();
                            setPathState(12);
                        }
                    }
                }
                break;
//            case 12: //go to wall human player
//                if (!follower.isBusy()) {
//                    if (init){
//                        follower.setMaxPower(1);
//                        //intake.setFlickerPosition(Intake.FLICKER_OPEN_POSITION);

//                        init = false;
//                    }
//                    else{
//                        follower.followPath(goToWallHumanPlayer, true);
//                        setPathState(13);
//                    }
//                }
//                break;
//            case 13: //pickup human player
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickupHumanPlayer, true);
//                    setPathState(14);
//                }
//                break;
//            case 14: //move to score position for human player
//                if (!follower.isBusy() || opmodeTimer.getElapsedTimeSeconds() > 26) {
//                    if (init){
//                        intake.turnOffIntake();
//                        init = false;
//                    }
//                    else{
//                        follower.followPath(scorePickupHumanPlayer, true);
//                        intializeBurstClose(); //prestart shooter
//                        turnOnShooterAuto();
//                        setPathState(15);
//                    }
//                }
//                break;
//            case 15: //score human player
//                if (!follower.isBusy()) {
//                    if(init){
////                            intializeBurstClose();
////                            turnOnShooterAuto();
//                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);
//
//                        init = false;
//                    }
//                    else{
//                        if (pathTimer.getElapsedTimeSeconds() > INTAKE_DELAY_TIME - 0.4) {
//                            intake.turnOnIntakeAuto();
//                            //intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);

//                        }
//
//                        if (shooter.ballsShot >= 15 || opmodeTimer.getElapsedTimeSeconds() > 29.7) {
//                            scorePickupHumanPlayerTime = opmodeTimer.getElapsedTimeSeconds();
//
//                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
//                            turnOffShooterAuto();
//                            setPathState(16);
//                        }
//                    }
//                }
//                break;
            case 12:
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
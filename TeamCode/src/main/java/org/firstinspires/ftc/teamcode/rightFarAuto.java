package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;


@Config
@Autonomous(name="Red Far Auto", group="Robot")
@SuppressWarnings("FieldCanBeLocal")
public class rightFarAuto extends LinearOpMode {
    Drivetrain drivetrain;
    Shooter shooter;
    Intake intake;
//    Camera camera;
    private Follower follower; //initialize the follower object
    private Timer pathTimer, opmodeTimer; //declare the time variables used when checking for path completion
    private Pose currentPose;
    private int pathState = 0; //finite state machine variable
    private boolean init = true;
    public static double FIRST_INTAKE_DELAY_TIME = 1.5;
    public static double FOLLOWING_INTAKE_DELAY_TIME = 0.5;

    public static double SHOOTER_ON_DELAY_PRELOAD = 0.9; //prevent motors from revving up too much when shooting preloads
    public static double RELEASE_BALLS_WAIT_TIME = 0.15; //time to wait at the chamber
    public static double HEADING_INTERPOLATION_END_PERCENTAGE = 0.65;
    public static double AUTO_Y_OFFSET = 0;
    public static double RELEASE_BALLS_Y = 73.5;
    public static double INTAKE_X_OFFSET = 0.5;
    public static double SCORE_HEADING_OFFSET = 0; //score heading offset since center of goals are not exactly 45 degrees
    public static double MAX_POWER = 1;

    //variables to keep track of how long each score took in order to implement failsafes based on the opmode timer
    private double scorePreloadTime = 0.0;
    private double scorePickupTopTime = 0.0;
    private double scorePickupMiddleTime = 0.0;
    private double scorePickupBottomTime = 0.0;
    public double scoreHeading = 67;

    private PathChain scorePreload, collectHumanPlayer, scoreHumanPlayer, goToPark; //define path chains (muliple paths interpolated)

    private final Pose startPose = new Pose(89.5, 8+AUTO_Y_OFFSET, Math.toRadians(90)); // Start Pose of our robot
    private final Pose scorePose =  new Pose(83,14, Math.toRadians(scoreHeading));
    private final Pose collectHumanPlayerPose = new Pose(131, 8, Math.toRadians(0));
    private final Pose moveBackHumanPlayerPose = new Pose(127, 8, Math.toRadians(0));
    private final Pose parkPose = new Pose(100,14, Math.toRadians(0));

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        collectHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, collectHumanPlayerPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), collectHumanPlayerPose.getHeading())
                .setTimeoutConstraint(250)

                .addPath(new BezierLine(collectHumanPlayerPose, moveBackHumanPlayerPose))
                .setConstantHeadingInterpolation(collectHumanPlayerPose.getHeading())
                .setTimeoutConstraint(250)

                .addPath(new BezierLine(moveBackHumanPlayerPose, collectHumanPlayerPose))
                .setConstantHeadingInterpolation(collectHumanPlayerPose.getHeading())
                .setTimeoutConstraint(250)

                .build();
        scoreHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(collectHumanPlayerPose, scorePose))
                .setLinearHeadingInterpolation(collectHumanPlayerPose.getHeading(), scorePose.getHeading())
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

            PoseManager.initializeTeleopPoses(PoseManager.Team.RED, currentPose);

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


            telemetry.update();

        }
    }

    public void updateStateMachine() {
        switch (pathState) {
            case 0: //move to score position for preload
                if (init){
                    init = false;
                }
                else{ //move to scoring position
                    follower.followPath(scorePreload, true);
                    shooter.initializeBurstFar(); //prestart shooter
                    shooter.turnOnShooterAuto();
                    setPathState(1);
                }
                break;
            case 1: //score preloads
                if (!follower.isBusy()) {
                    if(init){
                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                        init = false;
                    }
                    else{
                        if (pathTimer.getElapsedTimeSeconds() > FIRST_INTAKE_DELAY_TIME) {
                            intake.turnOnIntakeAutoFar();
                        }

                        if (pathTimer.getElapsedTimeSeconds() > AutoOverrideTimes.OVERRIDE_FAR_SHOOT_TIME) {
                            scorePreloadTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 3;
                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
//                            turnOffShooterAuto();
                            setPathState(2); //end
                        }
                    }
                }
                break;
            case 2: // intake human player
                if (!follower.isBusy()) {
                    if (init){
                        shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                        intake.turnOnIntakeAuto();
                        init = false;
                    }
                    else{
                        follower.followPath(collectHumanPlayer, false);
                        setPathState(3);
                    }
                }
                break;
            case 3: //move to score human player
                if (!follower.isBusy()) {
                    if (init){
                        init = false;
                    }
                    else{
                        follower.followPath(scoreHumanPlayer, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    if(init){
                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                        init = false;
                    }
                    else{
                        if (pathTimer.getElapsedTimeSeconds() > FOLLOWING_INTAKE_DELAY_TIME) {
                            intake.turnOnIntakeAutoFar();
                        }

                        if (pathTimer.getElapsedTimeSeconds() > AutoOverrideTimes.OVERRIDE_FAR_SHOOT_TIME) {
                            scorePreloadTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 6;
                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
//                            turnOffShooterAuto(); don't turn off shooter to keep moment of intertia and speed
                            setPathState(5); //end
                        }
                    }
                }
                break;
            case 5: // intake human player
                if (!follower.isBusy()) {
                    if (init){
                        shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                        intake.turnOnIntakeAuto();
                        init = false;
                    }
                    else{
                        follower.followPath(collectHumanPlayer, false);
                        setPathState(6);
                    }
                }
                break;
            case 6: //move to score human player
                if (!follower.isBusy()) {
                    if (init){
                        init = false;
                    }
                    else{
                        follower.followPath(scoreHumanPlayer, true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    if(init){
                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                        init = false;
                    }
                    else{
                        if (pathTimer.getElapsedTimeSeconds() > FOLLOWING_INTAKE_DELAY_TIME) {
                            intake.turnOnIntakeAutoFar();
                        }

                        if (shooter.ballsShot >= 9 || pathTimer.getElapsedTimeSeconds() > AutoOverrideTimes.OVERRIDE_FAR_SHOOT_TIME) {
                            scorePreloadTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 9;
                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
//                            turnOffShooterAuto(); don't turn off shooter to keep moment of intertia and speed
                            setPathState(8); //end
                        }
                    }
                }
                break;
            case 8: // intake human player
                if (!follower.isBusy()) {
                    if (init){
                        shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                        intake.turnOnIntakeAuto();
                        init = false;
                    }
                    else{
                        follower.followPath(collectHumanPlayer, false);
                        setPathState(9);
                    }
                }
                break;
            case 9: //move to score human player
                if (!follower.isBusy()) {
                    if (init){
                        init = false;
                    }
                    else{
                        follower.followPath(scoreHumanPlayer, true);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    if(init){
                        shooter.setGatePosition(Shooter.GATE_OPEN_POSITION);

                        init = false;
                    }
                    else{
                        if (pathTimer.getElapsedTimeSeconds() > FOLLOWING_INTAKE_DELAY_TIME) {
                            intake.turnOnIntakeAutoFar();
                        }

                        if (pathTimer.getElapsedTimeSeconds() > AutoOverrideTimes.OVERRIDE_FAR_SHOOT_TIME) {
                            scorePreloadTime = opmodeTimer.getElapsedTimeSeconds();

                            shooter.ballsShot = 12;
                            shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                            shooter.turnOffShooterAuto(); //turn off shooter for last cycle
                            setPathState(11); //end
                        }
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    follower.followPath(goToPark);
                    setPathState(-1);
                }
                break;
        } //run state machine
    }

    public void setPathState(int pState) {
        pathState = pState;
        init = true;
        pathTimer.resetTimer();
        shooter.resetShooterClosedTimer();
        shooter.resetShooterOpenTimer();
    }

}
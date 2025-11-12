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
@Autonomous(name="Blue Close Auto From Far", group="Robot")
@SuppressWarnings("FieldCanBeLocal")
public class leftCloseAutoFromFar extends LinearOpMode {
    Drivetrain drivetrain;
    Shooter shooter;
    Intake intake;
    Camera camera;
    private Follower follower; //initialize the follower object
    private Timer pathTimer, opmodeTimer; //declare the time variables used when checking for path completion
    private Pose currentPose;
    private int pathState = 0; //finite state machine variable
    private boolean init = true;
    public static double INTAKE_DELAY_TIME = 3;
    public static double SCORE_HEADING_OFFSET = 5; //score heading offset since center of goals are not exactly 45 degrees

    public double scoreHeading = Math.toRadians(135 + SCORE_HEADING_OFFSET);

    private PathChain scorePreload, grabPickupBottom, scorePickupBottom, grabPickupMiddle, scorePickupMiddle, grabPickupTop, scorePickupTop; //define path chains (muliple paths interpolated)

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot
    private final Pose scorePose = new Pose(58, 90, scoreHeading); //TODO: change this to not be middle
    private final Pose grabPickupTopPose = new Pose(16, 84, Math.toRadians(180));
    private final Pose grabPickupTopPoseControlPoint1 = new Pose(60.923, 85.514);


    //TODO: SET OTHER POSES


    //BUILD PATHS TODO: actually build them using poses from visualizer
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickupTop = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose, grabPickupTopPoseControlPoint1, grabPickupTopPose))
                .addPath(new BezierLine(scorePose, grabPickupTopPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabPickupTopPose.getHeading())
                .addPoseCallback(new Pose(20, 84), intake::closeFlicker, 0.5)
                .build();
        scorePickupTop = follower.pathBuilder()
                .addPath(new BezierLine(grabPickupTopPose, scorePose))
                .setLinearHeadingInterpolation(grabPickupTopPose.getHeading(), scorePose.getHeading())
                .build();
//        grabPickupMiddle = follower.pathBuilder()
//                .build();
//        scorePickupMiddle = follower.pathBuilder()
//                .build();
//        grabPickupTop = follower.pathBuilder()
//                .build();
//        scorePickupTop = follower.pathBuilder()
//                .build();
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
                            setPathState(-1); //end
                        }
                    }
                }
                break;
            case 2: // intake top row
                if (!follower.isBusy()) {

                    follower.followPath(grabPickupTop);
                    setPathState(3);
                }
                break;
            case 3: //move to score position for top row
                if (!follower.isBusy()) {
                    follower.followPath(scorePickupTop);

                    setPathState(4);
                }
                break;
            case 4: //score top row
                if (!follower.isBusy()) {
                    intializeBurstClose();
//                    burstShoot();

                    if (shooter.ballsShot >= 6) {
                        shooter.turnOffShooter();
                        setPathState(-1); //end
                    }
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
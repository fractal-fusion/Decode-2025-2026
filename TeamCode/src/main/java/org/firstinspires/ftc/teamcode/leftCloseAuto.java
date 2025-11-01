package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Blue Close Auto", group="Robot")
@SuppressWarnings("FieldCanBeLocal")
public class leftCloseAuto extends LinearOpMode {
    private Follower follower; //initialize the follower object
    private Timer pathTimer, opmodeTimer; //declare the time variables used when checking for path completion
    private Pose currentPose;
    private int pathState; //finite state machine variable

    private PathChain scorePreload, grabPickupBottom, scorePickupBottom, grabPickupMiddle, scorePickupMiddle, grabPickupTop, scorePickupTop; //define path chains (muliple paths interpolated)

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot
    private final Pose scorePreloadPose = new Pose(72, 84, Math.toRadians(135));

    //TODO: SET OTHER POSES


    //BUILD PATHS TODO: actually build them using poses from visualizer
    public void buildPaths(){
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
                .build();
//        grabPickupBottom = follower.pathBuilder()
//                .build();
//        scorePickupBottom = follower.pathBuilder()
//                .build();
//        grabPickupMiddle = follower.pathBuilder()
//                .build();
//        scorePickupMiddle = follower.pathBuilder()
//                .build();
//        grabPickupTop = follower.pathBuilder()
//                .build();
//        scorePickupTop = follower.pathBuilder()
//                .build();
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize subsystems
        Drivetrain drivetrain = new Drivetrain(this);
        Shooter shooter = new Shooter(this);
        Intake intake = new Intake(this, Intake.FLICKER_CLOSE_POSITION);
        Camera camera = new Camera(this, 3);

        //initialize timers so they can be checked in the state machine
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap); //create pedropathing follower
        follower.setStartingPose(startPose);
        follower.setMaxPower(1); //decrease max power to prevent flipping

        buildPaths(); //build all paths

        if (isStopRequested()) return;

        waitForStart();

        opmodeTimer.resetTimer(); //reset opmode total timer on start

        setPathState(0);

        while (opModeIsActive()){
            follower.update(); //update follower
            currentPose = follower.getPose(); //update current pose

            switch (pathState) {
                case 0:
                    //hold the flicker in
                    intake.setFlickerPosition(Intake.FLICKER_CLOSE_POSITION);
                    shooter.setPitchPosition(Shooter.PITCH_INTAKE_POSITION);


                    // Move to the scoring position from the start position
                    follower.followPath(scorePreload, true);
                    setPathState(1);
                    break;
                case 1:
                    // Wait until we have passed all path constraints
                    if (!follower.isBusy()) {
                        shooter.setCurrentTargetRPMTicksPerSecond(Shooter.CLOSE_TARGET_RPM);
                        shooter.setRampPosition(Shooter.CLOSE_RAMP_SCORE_POSITION);

                        shooter.turnOnShooter();
                        intake.turnOnIntake();

                        if (shooter.shooterAtTargetVelocity() && shooter.checkPitchDebounceTimer()) {
                            shooter.setPitchPosition(Shooter.PITCH_SCORE_POSITION);
                            shooter.resetPitchTimer();
                        }
                        else if (!shooter.shooterAtTargetVelocity()){
                            shooter.setPitchPosition(Shooter.PITCH_INTAKE_POSITION);
                        }
                        // move to the first artifact pickup location from the scoring position
//                    follower.followPath(grabPickupBottom);

                        if (opmodeTimer.getElapsedTimeSeconds() > 10) {
                            setPathState(-1); //end
                        }
                    }
                    break;
            } //run state machine
            shooter.updatePitchDebounceTimer(); //update pitch timer for staying down between each ball
        }
    }
}

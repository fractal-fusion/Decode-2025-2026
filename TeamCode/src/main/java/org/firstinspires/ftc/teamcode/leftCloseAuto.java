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
public class leftCloseAuto extends LinearOpMode {
    //initialize subsystems
    Drivetrain drivetrain = new Drivetrain(this);
    Shooter shooter = new Shooter(this);
    Intake intake = new Intake(this);
    Camera camera = new Camera(this, 3);

    private Follower follower; //initialize the follower object
    private Timer pathTimer, actionTimer, opmodeTimer; //declare the variables used when checking for path completion
    private int pathState; //finite state machine variable

    private PathChain scorePreload, grabPickupBottom, scorePickupBottom, grabPickupMiddle, scorePickupMiddle, grabPickupTop, scorePickupTop; //define path chains (muliple paths interpolated)

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    //TODO: SET OTHER POSES


    //BUILD PATHS TODO: actually build them using poses from visualizer
    public void buildPaths(){
        scorePreload = follower.pathBuilder()
                .build();
        grabPickupBottom = follower.pathBuilder()
                .build();
        scorePickupBottom = follower.pathBuilder()
                .build();
        grabPickupMiddle = follower.pathBuilder()
                .build();
        scorePickupMiddle = follower.pathBuilder()
                .build();
        grabPickupTop = follower.pathBuilder()
                .build();
        scorePickupTop = follower.pathBuilder()
                .build();
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void updatePathState(){
        switch (pathState) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(scorePreload);

                //TODO: LOGIC FOR SCORING HERE or add a callback to run while following a path

                setPathState(1);
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // move to the first artifact pickup location from the scoring position
                    follower.followPath(grabPickupBottom);
                    setPathState(2);
                }
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize timers and followers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        buildPaths();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()){
            updatePathState();
        }
    }
}

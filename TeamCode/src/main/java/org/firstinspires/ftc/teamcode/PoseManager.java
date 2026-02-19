package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

public class PoseManager {
    public enum Team {
        NONE,
        BLUE,
        RED
    }

    public static double FAR_SHOOTING_HEADING_RED = 70;

    public static Pose currentPose = new Pose();
//    public static Team currentTeam = Team.NONE;

    //initialize where the team-specific poses actually are
    public static Pose RED_GOAL_POSITION = new Pose(144, 140);
    public static Pose BLUE_GOAL_POSITION = RED_GOAL_POSITION.mirror();

    public static Pose RED_LEVER_POSITION = new Pose(134,61.5, Math.toRadians(40.5));
    public static Pose BLUE_LEVER_POSITION = RED_LEVER_POSITION.mirror();

    public static Pose RED_FAR_POSITION = new Pose(83,14, Math.toRadians(FAR_SHOOTING_HEADING_RED));
    public static Pose BLUE_FAR_POSITION = RED_FAR_POSITION.mirror();

    //define where team-specific poses are stored
    public static Pose currentGoalPose = RED_GOAL_POSITION; //start on red in case teleop is started without first starting auto
    public static Pose currentLeverPose = RED_LEVER_POSITION;
    public static Pose currentFarPose = RED_FAR_POSITION;


    //initalize and store the team-specific poses into the correct storage variables, as well as store the current pose
    public static void initializeTeleopPoses(Team team, Pose currentpose){
        if (team == Team.BLUE) {
            currentGoalPose = BLUE_GOAL_POSITION;
            currentLeverPose = BLUE_LEVER_POSITION;
            currentFarPose = BLUE_FAR_POSITION;
        }
        else if (team == Team.RED) {
            currentGoalPose = RED_GOAL_POSITION;
            currentLeverPose = RED_LEVER_POSITION;
            currentFarPose = RED_FAR_POSITION;
        }

        currentPose = currentpose;
    }
}

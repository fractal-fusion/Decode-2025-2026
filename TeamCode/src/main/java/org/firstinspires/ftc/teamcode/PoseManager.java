package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class PoseManager {
    public enum Team {
        NONE,
        BLUE,
        RED
    }

    public static double RED_FAR_SHOOTING_HEADING = 70;
    public static double RED_LEVER_HEADING = 40.5;
//    public static double BLUE_FAR_SHOOTING_HEADING = 70 + 45;
//    public static double BLUE_LEVER_HEADING = RED_LEVER_HEADING + 90;

    public static Pose currentPose = new Pose();
//    public static Team currentTeam = Team.NONE;

    //initialize where the team-specific poses actually are
    public static Pose RED_GOAL_DISTANCE_POSITION = new Pose(130, 130);
    public static Pose BLUE_GOAL_DISTANCE_POSITION = RED_GOAL_DISTANCE_POSITION.mirror();

    public static double RED_GOAL_AUTOALIGN_X = 140;
    public static double RED_GOAL_AUTOALIGN_Y = 137;

    public static Pose RED_GOAL_AUTOALIGN_POSITION = new Pose(RED_GOAL_AUTOALIGN_X, RED_GOAL_AUTOALIGN_Y);
    public static Pose BLUE_GOAL_AUTOALIGN_POSITION = new Pose(RED_GOAL_AUTOALIGN_X, RED_GOAL_AUTOALIGN_Y).mirror();

    public static Pose RED_LEVER_POSITION = new Pose(134.5,60.8, Math.toRadians(RED_LEVER_HEADING));
    public static Pose BLUE_LEVER_POSITION = RED_LEVER_POSITION.mirror();

    public static Pose RED_FAR_POSITION = new Pose(83,14, Math.toRadians(RED_FAR_SHOOTING_HEADING));
    public static Pose BLUE_FAR_POSITION = RED_FAR_POSITION.mirror();

    //define where team-specific poses are stored
    public static Pose currentGoalPose = RED_GOAL_DISTANCE_POSITION; //start on red in case teleop is started without first starting auto
    public static Pose currentGoalAutoAlignPose = RED_GOAL_AUTOALIGN_POSITION;
    public static Pose currentLeverPose = RED_LEVER_POSITION;
    public static Pose currentFarPose = RED_FAR_POSITION;


    //initalize and store the team-specific poses into the correct storage variables, as well as store the current pose
    public static void initializeTeleopPoses(Team team, Pose currentpose){
        if (team == Team.BLUE) {
            currentGoalPose = BLUE_GOAL_DISTANCE_POSITION;
            currentGoalAutoAlignPose = BLUE_GOAL_AUTOALIGN_POSITION;
            currentLeverPose = BLUE_LEVER_POSITION;
            currentFarPose = BLUE_FAR_POSITION;
        }
        else if (team == Team.RED) {
            currentGoalPose = RED_GOAL_DISTANCE_POSITION;
            currentGoalAutoAlignPose = RED_GOAL_AUTOALIGN_POSITION;
            currentLeverPose = RED_LEVER_POSITION;
            currentFarPose = RED_FAR_POSITION;
        }

        currentPose = currentpose;
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class PoseManager {
    public enum Team {
        NONE,
        BLUE,
        RED
    }

    public static Pose currentPose = new Pose();
//    public static Team currentTeam = Team.NONE;

    //define where team-specific poses are stored
    public static Pose currentGoalPose = new Pose();

    //initialize where the team-specific poses actually are
    public static Pose BLUE_GOAL_POSITION = new Pose(0, 144);
    public static Pose RED_GOAL_POSITION = new Pose(144, 144);


    //initalize and store the team-specific poses into the correct storage variables, as well as store the current pose
    public static void initializeTeleopPoses(Team team, Pose currentpose){
        if (team == Team.BLUE) {
            currentGoalPose = BLUE_GOAL_POSITION;
        }
        else if (team == Team.RED) {
            currentGoalPose = RED_GOAL_POSITION;
        }

        currentPose = currentpose;
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Collections;

@Config
public class Limelight {

    private OpMode opMode;
    public Limelight3A limelight;

    private final int GPP_id = 21;
    private final int PGP_id = 22;
    private final int PPG_id = 23;

    LowPassFilter xFilter = new LowPassFilter(0.5);
    LowPassFilter yFilter = new LowPassFilter(0.5);


    public static double HEADING_FAR_RANGE_THRESHOLD = 0.61; //TODO: need to retune this
    public static double HEADING_OFFSET_CLOSE = 2.5; //offset for autoalign
    public static double HEADING_OFFSET_FAR = 0.5;
    public static double HEADING_VALID_RANGE = 5; //valid heading range for regression and shooting to be 100% accurate

    public Timer relocalizationTimer;
    public static double RELOCALIZATION_INTERVAL_SECONDS = 10;
    public ArrayList<Pose> SamplePoses = new ArrayList<>();
    public static int MAX_RELOCALIZATION_SAMPLES = 6;
    public static int RELOCALIZATION_SAMPLE_THRESHOLD = 5; //how many samples is enough to relocalize
    public static double RELOCALIZATION_OUTLIER_THRESHOLD_INCHES = 8;

    public boolean isFar;

    public double headingOffset;

    public Limelight(OpMode linearOpMode) {
        //initialize opmode variable for current opmode
        this.opMode = linearOpMode;

        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        relocalizationTimer = new Timer();
    }
    public void telemetryAprilTag() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            opMode.telemetry.addData("Target X", tx);
            opMode.telemetry.addData("Target Y", ty);
            opMode.telemetry.addData("Target Area", ta);
        } else {
            opMode.telemetry.addData("Limelight", "No Targets");
        }
    }

//    public Color[] getMosaic() { //TODO: make this work for limelight with a separate pipeline filtered for mosaic apriltags
//        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) { //get all detections
//            if (detection.id > 0) { //check if a recognized apriltag was detected (-1 is an unrecognized apriltag)
//                switch (detection.id) { //check which mosaic pattern and return a list of the colors
//                    case GPP_id:
//                        return new Color[]{Color.GREEN, Color.PURPLE, Color.PURPLE};
//                    case PGP_id:
//                        return new Color[]{Color.PURPLE, Color.GREEN, Color.PURPLE};
//                    case PPG_id:
//                        return new Color[]{Color.PURPLE, Color.PURPLE, Color.GREEN};
//                    default:
//                        return null; //return null if detected apriltag is not a mosaic apriltag
//                }
//            }
//        }
//        return null;
//    }
//
    public double getBearing() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0.0;
    }

    public double getRange(){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            return result.getTa();
        }

        return 0.0;
    }

    public void updateIsFar(){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            isFar = result.getTa() < HEADING_FAR_RANGE_THRESHOLD;
        }
    }

    public boolean isValidResult(){
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public boolean isReadyToRelocalize(){
        return relocalizationTimer.getElapsedTimeSeconds() > RELOCALIZATION_INTERVAL_SECONDS && SamplePoses.size() >= RELOCALIZATION_SAMPLE_THRESHOLD;
    }

    public Pose getSamplePose(){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D limelightPoseM1 = result.getBotpose();
            Pose3D limelightPoseM2 = result.getBotpose_MT2();


            Position limelightPoseInches = limelightPoseM2.getPosition().toUnit(DistanceUnit.INCH);
            double x = limelightPoseInches.y + 72;
            double y = -limelightPoseInches.x + 72;

            limelight.updateRobotOrientation(limelightPoseM1.getOrientation().getYaw());

            double yaw = Math.toRadians(limelightPoseM1.getOrientation().getYaw() - 90);
            return new Pose(xFilter.filter(x), yFilter.filter(y), yaw);

        }

        return new Pose();
    }

    public Pose getFilteredPose(Follower follower){

        //obtain all x and y values
        ArrayList<Double> poseXValues = new ArrayList<>();
        ArrayList<Double> poseYValues = new ArrayList<>();
        for (Pose samplePose : SamplePoses) {
            poseXValues.add(samplePose.getX());
            poseYValues.add(samplePose.getY());
        }

        //find medians
        Collections.sort(poseXValues);
        Collections.sort(poseYValues);

        double medianX = poseXValues.get(Math.floorDiv(poseXValues.size(), 2));
        double medianY = poseYValues.get(Math.floorDiv(poseYValues.size(), 2));

        //filter poses
        ArrayList<Pose> filteredPoses = new ArrayList<>();

        for (Pose samplePose : SamplePoses) {
            if (Math.abs(samplePose.getX() - medianX) <= RELOCALIZATION_OUTLIER_THRESHOLD_INCHES && Math.abs(samplePose.getY() - medianY) <= RELOCALIZATION_OUTLIER_THRESHOLD_INCHES) {
                filteredPoses.add(samplePose);
            }
        }

        //calculate average
        double xSum = 0;
        double ySum = 0;
        double sinSum = 0;
        double cosSum = 0;
        for (Pose filteredPose : filteredPoses) {
            xSum += filteredPose.getX();
            ySum += filteredPose.getY();
            sinSum += Math.sin(filteredPose.getHeading());
            cosSum += Math.cos(filteredPose.getHeading());
        }
        Pose averagePose = new Pose(xSum/filteredPoses.size(), ySum/filteredPoses.size(), follower.getHeading());

        if (filteredPoses.isEmpty()) {
            return new Pose(); //return nothing to prevent divide by zero
        }

        return averagePose;
    }

    public void updateFilteredPoseSamples(){
        if (SamplePoses.size() < MAX_RELOCALIZATION_SAMPLES){
            SamplePoses.add(getSamplePose());
        }
        else{
            SamplePoses.remove(0);
            SamplePoses.add(getSamplePose());
        }
    }

    public void clearSamplePoses(){
        SamplePoses.clear();
    }

    public void resetRelocalizationTimer(){
        relocalizationTimer.resetTimer();
    }

//    public void setHeadingOffset(double offset){
//        headingOffset = offset;
//    }
    private static class LowPassFilter {
        private double filterGain;
        private double smoothedValue = 0;

        public LowPassFilter(double gain){
            this.filterGain = gain;
        }

        public double filter(double newValue){
            smoothedValue = (filterGain * newValue) + ((1 - filterGain) * smoothedValue);
            return smoothedValue;
        }
}

}

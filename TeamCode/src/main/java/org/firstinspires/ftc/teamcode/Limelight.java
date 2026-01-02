package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Limelight {

    private OpMode opMode;
    public Limelight3A limelight;

    private final int GPP_id = 21;
    private final int PGP_id = 22;
    private final int PPG_id = 23;

    public static double HEADING_FAR_RANGE_THRESHOLD = 0.61; //TODO: need to retune this
    public static double HEADING_OFFSET_CLOSE = 2.5; //offset for autoalign
    public static double HEADING_OFFSET_FAR = 0.5;
    public static double HEADING_VALID_RANGE = 5; //valid heading range for regression and shooting to be 100% accurate

    public boolean isFar;

    public double headingOffset;

    public Limelight(OpMode linearOpMode) {
        //initialize opmode variable for current opmode
        this.opMode = linearOpMode;

        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
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
            return result.getTy();
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

    public void setHeadingOffset(double offset){
        headingOffset = offset;
    }
}

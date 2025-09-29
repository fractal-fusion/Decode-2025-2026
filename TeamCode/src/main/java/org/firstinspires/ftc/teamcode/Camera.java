package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Camera {

    private OpMode opMode;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

//    private Color[]  GPP = {Color.GREEN, Color.PURPLE, Color.PURPLE};
//    private Color[]  PGP = {Color.PURPLE, Color.GREEN, Color.PURPLE};
//    private Color[]  PPG = {Color.PURPLE, Color.PURPLE, Color.GREEN};

    private final int GPP_id = 21;
    private final int PGP_id = 22;
    private final int PPG_id = 23;

    public Camera(OpMode linearOpMode) {
        //initialize opmode variable for current opmode
        this.opMode = linearOpMode;

        //create april tag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // C310 has preset callibrated values in the sdk

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);

        //use builder pattern to configure vision portal settings
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTagProcessor);

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        //create the vision portal
        visionPortal = builder.build();


    }
    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (i" + "nch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to opMode.telemetry
        opMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        opMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        opMode.telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    public Color[] getMosaic() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) { //get all detections
            if (detection.id > 0) { //check if a recognized apriltag was detected (-1 is an unrecognized apriltag)
                switch (detection.id) { //check which mosaic pattern and return a list of the colors
                    case GPP_id:
                        return new Color[]{Color.GREEN, Color.PURPLE, Color.PURPLE};
                    case PGP_id:
                        return new Color[]{Color.PURPLE, Color.GREEN, Color.PURPLE};
                    case PPG_id:
                        return new Color[]{Color.PURPLE, Color.PURPLE, Color.GREEN};
                    default:
                        return null; //return null if detected apriltag is not a mosaic apriltag
                }
            }
        }
        return null;
    }
    
    public double getBearing() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id > 0){
                if (detection.id == 20 || detection.id == 24) { //check for recognized blue (20) or red (24) alliance apriltag
                    return detection.ftcPose.bearing;
                }
            }
        }

        return 0.0;
    }

    public void turnOnCamera(){
        visionPortal.resumeStreaming();
    }

    public void turnOffCamera(){
        visionPortal.stopStreaming();
    }

    public void setExposure(int exposureMs) {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
        }
    }

    //TODO: make methods for detecting a specific apriltag, detecting the current pattern, and getting the pose
    
}

//package org.firstinspires.ftc.teamcode.common.hardware;
//
//import android.util.Size;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//import global.first.FeedingTheFutureGameDatabase;
//
////@Config
//public class RobotHardware {
//
//    private static RobotHardware instance = null;
//    private boolean enabled = false;
//
//    public static RobotHardware getInstance() {
//        if (instance == null) {
//            instance = new RobotHardware();
//        }
//        instance.enabled = true;
//        return instance;
//    }
//
//    private HardwareMap hardwareMap;
//
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//    private final String webCamName = "WebCamFGC";
//
//    public void init(final HardwareMap hardwareMap) {
//        this.hardwareMap = hardwareMap;
//
//        startCamera();
//    }
//
//    public void startCamera() {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, webCamName))
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .addProcessors(aprilTag)
//                .enableLiveView(true)
//                .build();
//
//        visionPortal.setProcessorEnabled(aprilTag, true);
//    }
//
//    public Pose2d getApriltagDetection() {
//        if(aprilTag != null) {
//            List<AprilTagDetection> detections = aprilTag.getDetections();
//            for (AprilTagDetection detection : detections) {
//                if(detection.metadata != null) {
//
//                }
//            }
//            return new Pose2d();
//        }
//        else {
//            return null;
//        }
//    }
//
//
//}

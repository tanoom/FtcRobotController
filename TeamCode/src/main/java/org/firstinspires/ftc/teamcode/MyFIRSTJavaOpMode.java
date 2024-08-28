package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameInternal;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import global.first.FeedingTheFutureGameDatabase;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.tensorflow.lite.Tensor;

import java.util.List;

@TeleOp(name = "My FIRST Java OpMode", group = "Iterative Opmode")
public class MyFIRSTJavaOpMode extends OpMode {
    private TelemetryPacket packet = new TelemetryPacket();
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final String webCamName = "WebCamFGC";

    @Override
    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webCamName))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();
        visionPortal.setProcessorEnabled(aprilTag, true);

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webCamName), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    @Override
    public void loop() {
        //FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        getApriltagDetections();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public Pose2d getApriltagDetections() {
        if(aprilTag != null) {
            List<AprilTagDetection> tagList = aprilTag.getDetections();

            packet.put("Tag Count", tagList.size());

            for(AprilTagDetection detection : tagList) {
                VectorF position_tag_robot;
                VectorF rotation_tag_robot;

                VectorF position_tag_field = FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary().lookupTag(detection.id).fieldPosition;
                VectorF position_robot_field;

                packet.put("Tag ID", detection.id);
                VectorF vf = FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary().lookupTag(detection.id).fieldPosition;
                //dashboard.getTelemetry().addData("Tag FTCPose", detection.ftcPose);
                if(detection.ftcPose != null) {

                    position_tag_robot = new VectorF((float) detection.ftcPose.x, (float) detection.ftcPose.y, (float) detection.ftcPose.z);
                    rotation_tag_robot = new VectorF((float) detection.ftcPose.yaw, (float) detection.ftcPose.roll, (float) detection.ftcPose.pitch);
                    position_robot_field = position_tag_field.subtracted(position_tag_robot);




                    packet.put("Tag RawPoseX", detection.ftcPose.x);
                    packet.put("Tag RawPoseY", detection.ftcPose.y);
                    packet.put("Tag RawPoseZ", detection.ftcPose.z);
                    packet.put("Tag RawPoseYaw", detection.ftcPose.yaw);
                    packet.put("Tag RawPoseRoll", detection.ftcPose.roll);
                    packet.put("Tag RawPosePitch", detection.ftcPose.pitch);
                    //packet.put("Size of the vetor", vf.length());
                    //packet.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", vf.get(0), vf.get(1), vf.get(2)));
                }

                if(vf != null) {
                    packet.put("Tag PoseFieldX",vf.get(0));
                    packet.put("Tag PoseFieldY",vf.get(1));
                    packet.put("Tag PoseFieldZ",vf.get(2));
                }



            }
        }
        return new Pose2d();
    }
}

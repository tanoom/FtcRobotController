package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameInternal;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@TeleOp(name = "My FIRST Java OpMode", group = "Iterative Opmode")
public class MyFIRSTJavaOpMode extends OpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final String webCamName = "WebCamFGC";

    @Override
    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
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
    }

    public Pose2d getApriltagDetections() {
        if(aprilTag != null) {
            List<AprilTagDetection> tagList = aprilTag.getDetections();
            for(AprilTagDetection detection : tagList) {
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Tag FTCPose", detection.ftcPose);
                if(detection.rawPose != null) {
                    telemetry.addData("Tag RawPoseX", detection.rawPose.x);
                    telemetry.addData("Tag RawPoseY", detection.rawPose.y);
                    telemetry.addData("Tag RawPoseZ", detection.rawPose.z);
//                    for (int i = 0; i < detection.rawPose.R.numRows(); i++) {
//                        for (int j = 0; i < detection.rawPose.R.numCols(); j++) {
//                            telemetry.addData("Tag RawPoseRotations", i+" "+j+" "+detection.rawPose.R.get(i, j));
//                        }
//                    }
                }


            }
        }
        return new Pose2d();
    }
}

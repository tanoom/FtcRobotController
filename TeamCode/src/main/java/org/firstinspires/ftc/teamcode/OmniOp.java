package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.util.Pose3d;
import org.firstinspires.ftc.teamcode.common.util.Rotation3d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import global.first.FeedingTheFutureGameDatabase;

@TeleOp(name="OmniOp for WebCam", group="Linear OpMode")
public class OmniOp extends LinearOpMode {

    private TelemetryPacket packet = new TelemetryPacket();
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final String webCamName = "WebCamFGC";

    private final Pose3d cameraPoseOnRobot = new Pose3d();
    private final FtcDashboard ftcDashboard = FtcDashboard.getInstance();

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private IMU imu = null;
    private double angleOffset = 0;
    private boolean isFieldRelative = true;

    public void startCamera(){
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

        ftcDashboard.startCameraStream(visionPortal, 0);
    }


    public Pose2d getApriltagDetections() {
        if(aprilTag != null) {
            List<AprilTagDetection> tagList = aprilTag.getDetections();

            packet.put("Tag Count", tagList.size());

            for(AprilTagDetection detection : tagList) {
                Pose3d apriltagToFieldPose = new Pose3d();

                packet.put("Tag ID", detection.id);
                VectorF vf = FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary().lookupTag(detection.id).fieldPosition;
                //dashboard.getTelemetry().addData("Tag FTCPose", detection.ftcPose);
                if(detection.ftcPose != null) {
                    Pose3d tagToCameraPose = new Pose3d(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z,
                            new Rotation3d( detection.ftcPose.roll,
                                    detection.ftcPose.pitch,
                                    detection.ftcPose.yaw));
                    Pose3d robotToField3d = apriltagToFieldPose.transformBy(tagToCameraPose.toTransform3d().inverse())
                            .transformBy(cameraPoseOnRobot.toTransform3d().inverse());
                    Pose2d robotToField2d = robotToField3d.toPose2d();




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

    @Override
    public void runOpMode() {

        startCamera();

        packet.fieldOverlay()
                .setFill("blue")
                .fillRect(-20, -20, 40, 40);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        imu = hardwareMap.get(IMU.class, "imu");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        runtime.reset();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a) {
                imu.resetYaw();
            }

            if(gamepad1.left_bumper) {
                isFieldRelative = false;
            }else {
                isFieldRelative = true;
            }


            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double x   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double y =  -gamepad1.left_stick_x;


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

//            if(isFieldRelative) {
//                axial = axial * Math.cos(angleDegree) - lateral * Math.sin(angleDegree);
//                lateral = axial * Math.sin(angleDegree) + lateral * Math.cos(angleDegree);
//            }

            double yaw     =  -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = rotY + rotX + yaw;
            double rightFrontPower = rotY - rotX - yaw;
            double leftBackPower   = rotY - rotX + yaw;
            double rightBackPower  = rotY + rotX - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            getApriltagDetections();
            packet.put("Tag Count", 1);

            ftcDashboard.sendTelemetryPacket(packet);

//            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.update();
        }
    }
}

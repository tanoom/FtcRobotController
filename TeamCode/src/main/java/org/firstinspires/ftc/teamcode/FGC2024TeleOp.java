package org.firstinspires.ftc.teamcode;;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.XCYBoolean;

import java.util.List;
import java.util.concurrent.TimeUnit;

import global.first.FeedingTheFutureGameDatabase;

@TeleOp(name="FGC2024TeleOp")

public class FGC2024TeleOp extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor strafeDrive = null;
    private DcMotor frontSlide = null;
    private DcMotor backSlide = null;

    private Servo strafe = null;
    private Servo backDoor = null;
    private Servo frontDoor_left = null;
    private Servo frontDoor_right = null;

    private IMU imu = null;

    private static final boolean USE_WEBCAM = false;
    final double DESIRED_DISTANCE = 12.0;// Set true to use a webcam, or false for a phone camera
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private Pose2d current_pos;
    enum Sequence{
        INTAKE, RUN, RELEASE
    }
    private Sequence sequence;

    private TelemetryPacket packet = new TelemetryPacket();

    @Override public void runOpMode() throws InterruptedException
    {
        SuperStructure upper = new SuperStructure(this);
        boolean targetFound    = false;

        double drive = 0;
        double turn  =  0;

        double leftPower = 0;
        double rightPower = 0;

        boolean strafeDown = false;
        boolean openBackDoor = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        XCYBoolean dropStrafe = new XCYBoolean(()-> gamepad1.b);
        XCYBoolean moveLeft = new XCYBoolean(()-> gamepad1.dpad_left);
        XCYBoolean moveRight = new XCYBoolean(()-> gamepad1.dpad_right);

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        strafeDrive = hardwareMap.get(DcMotor.class,"strafe_drive");

        frontSlide = hardwareMap.get(DcMotor.class, "frontSlide");
        backSlide = hardwareMap.get(DcMotor.class,"backSlide");
        strafe = hardwareMap.get(Servo.class,"strafe");

        backDoor = hardwareMap.get(Servo.class,"doorBack");
        frontDoor_left = hardwareMap.get(Servo.class,"doorLeft");
        frontDoor_right = hardwareMap.get(Servo.class,"doorRight");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        strafeDrive.setDirection(DcMotor.Direction.FORWARD);

        frontSlide.setDirection(DcMotor.Direction.FORWARD);
        backSlide.setDirection(DcMotor.Direction.REVERSE);

        strafe.setDirection(Servo.Direction.FORWARD);
        backDoor.setDirection(Servo.Direction.FORWARD);
        frontDoor_left.setDirection(Servo.Direction.FORWARD);
        frontDoor_right.setDirection(Servo.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize the AprilTag Detection process
        initAprilTag();

        imu = hardwareMap.get(IMU.class, "imu");

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        while (opModeIsActive())
        {

            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            if(sequence == Sequence.RELEASE){

            }


            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound){
                if (gamepad1.left_bumper){
                    // Determine heading and range error so we can use them to control the robot automatically.
                    double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError = desiredTag.ftcPose.bearing;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

                    telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
                } else{
                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    drive = -gamepad1.left_stick_y / 2.0;
                    turn  =  -gamepad1.right_stick_x / 3.0;

                    telemetry.addData("Manual","Drive %5.2f, Turn %5.2f ", drive,turn);
                }
            } else{
                drive = -gamepad1.left_stick_y ;
                turn  =  -gamepad1.right_stick_x;
                leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            }


            frontSlide.setPower(gamepad2.right_trigger);
            backSlide.setPower(gamepad2.left_trigger);
            if(gamepad2.left_bumper){
                backSlide.setPower(-1);
            }
            if(gamepad2.right_bumper){
                frontSlide.setPower(-1);
            }

            if(gamepad2.a){
               upper.toLowRelease(SuperStructure.ReleaseLifter.BACK);
            }
            if(gamepad2.b){
                upper.toMidRelease(SuperStructure.ReleaseLifter.FRONT);
            }
            if(gamepad2.y){
                upper.toHighRelease(SuperStructure.ReleaseLifter.FRONT);
            }
            if(gamepad2.x){
                backDoor.setPosition(0.7);
            }

            if (gamepad1.b){
                strafeDown = true;
                strafe.setPosition(0.38);
//                strafe.setPosition(strafeDown?0.1:0.32);
            }
            if(gamepad1.x){
                strafe.setPosition(0.15);
            }

            frontDoor_left.setPosition((gamepad2.left_stick_y+1)/2);
            frontDoor_right.setPosition((gamepad2.right_stick_y+1)/2);


            if (moveLeft.toTrue()){
                strafeDrive.setPower(-1);
            } else if (moveRight.toTrue()) {
                strafeDrive.setPower(1);
            }else {
                strafeDrive.setPower(0);
            }

            if(gamepad1.dpad_left){
                strafeDrive.setPower(-1);
            }else if(gamepad1.dpad_right){
                strafeDrive.setPower(1);
            }else{
                strafeDrive.setPower(0);
        }
            moveRobot(drive,turn);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Send powers to the wheels.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
        // Send powers to the wheels.

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "WebCamFGC"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private double last_time_sec;
    private double period_time_sec;
//    private void logic_period() {
//        XCYBoolean.bulkRead();
//        current_pos = drive.getPoseEstimate();
//        period_time_sec = time.seconds() - last_time_sec;
//        telemetry.addData("elapse time", period_time_sec * 1000);
//        last_time_sec = time.seconds();
//        telemetry.update();
//    }

}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
@Config
public class slideTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int frontSlidePos = 0;
    public static int backSlidePos = 0;
    public static double max_power = 1;
    public static boolean read_only = false;
    public static boolean reverse_frontSlide = false;
    public static boolean reverse_backSlide = false;
    public static boolean reset = true;

    @Override
    public void runOpMode() {
        DcMotorEx frontLeftLift_Motor = hardwareMap.get(DcMotorEx.class, "frontLeftLift");
        DcMotorEx frontRightLift_Motor = hardwareMap.get(DcMotorEx.class, "frontRightLift");
//        DcMotorEx backLeftLift_Motor = hardwareMap.get(DcMotorEx.class, "backLeftLift");
//        DcMotorEx backRightLift_Motor = hardwareMap.get(DcMotorEx.class, "backRightLift");

        frontLeftLift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightLift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftLift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightLift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        
        if (reset) {
            frontLeftLift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftLift_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRightLift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightLift_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            backLeftLift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backLeftLift_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            backRightLift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backRightLift_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        if (reverse_frontSlide) {
            frontLeftLift_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(reverse_backSlide){
            frontRightLift_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        while (opModeIsActive()) {
            if (!read_only) {
//                frontLeftLift_Motor.setTargetPosition(frontSlidePos);
//                frontLeftLift_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightLift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontLeftLift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontLeftLift_Motor.setPower(max_power*(-gamepad1.left_stick_y));

//                frontRightLift_Motor.setTargetPosition(frontSlidePos);
//                frontRightLift_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightLift_Motor.setPower(max_power*(-gamepad1.left_stick_y));
            }
            telemetry_M.addData("is busy_frontLeftSlide", frontLeftLift_Motor.isBusy());
            telemetry_M.addData("is busy_frontRightSlide", frontRightLift_Motor.isBusy());
//            telemetry_M.addData("is busy_backLeftSlide", backLeftLift_Motor.isBusy());
//            telemetry_M.addData("is busy_backRightSlide", backRightLift_Motor.isBusy());

            telemetry_M.addData("encoder_frontLeftSlide", frontLeftLift_Motor.getCurrentPosition());
            telemetry_M.addData("encoder_frontRightSlide", frontRightLift_Motor.getCurrentPosition());
//            telemetry_M.addData("encoder_backLeftSlide", backLeftLift_Motor.getCurrentPosition());
//            telemetry_M.addData("encoder_backLeftSlide", backRightLift_Motor.getCurrentPosition());

            telemetry_M.update();
        }
    }
}

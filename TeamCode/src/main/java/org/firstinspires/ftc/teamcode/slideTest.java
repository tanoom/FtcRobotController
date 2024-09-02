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
    public static int frontSlidePos = 1850;
    public static int backSlidePos = 0;
    public static double max_power = 1;
    public static boolean read_only = false;
    public static boolean reverse_frontSlide = false;
    public static boolean reverse_backSlide = true;
    public static boolean reset = true;

    @Override
    public void runOpMode() {
        DcMotorEx frontSlideMotor = hardwareMap.get(DcMotorEx.class, "frontSlide");
        DcMotorEx backSlideMotor = hardwareMap.get(DcMotorEx.class, "backSlide");

        frontSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        
        if (reset) {
            frontSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        if (reverse_frontSlide) {
            frontSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(reverse_backSlide){
            backSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        while (opModeIsActive()) {
            if (!read_only) {
                frontSlideMotor.setTargetPosition(frontSlidePos);
                frontSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontSlideMotor.setPower(max_power);

                backSlideMotor.setTargetPosition(backSlidePos);
                backSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backSlideMotor.setPower(max_power);
            }
            telemetry_M.addData("is busy_frontSlide", frontSlideMotor.isBusy());
            telemetry_M.addData("is busy_backSlide", backSlideMotor.isBusy());

            telemetry_M.addData("encoder_frontSlide", frontSlideMotor.getCurrentPosition());
            telemetry_M.addData("encoder_backSlide", backSlideMotor.getCurrentPosition());
            telemetry_M.addData("velocity_frontSlide", frontSlideMotor.getVelocity());
            telemetry_M.addData("velocity_backSlide", backSlideMotor.getVelocity());
            telemetry_M.update();
        }
    }
}

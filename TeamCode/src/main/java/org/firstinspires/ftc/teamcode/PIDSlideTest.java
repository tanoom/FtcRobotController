package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PIDSlideTest")
@Config
public class PIDSlideTest extends LinearOpMode {
    //private TelemetryPacket packet = new TelemetryPacket();
    private final MultipleTelemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private static double slideHeightTicks = 0;
    private Motor mFrontSlide;
    private Motor mBackSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        mFrontSlide = new Motor(hardwareMap, "frontSlide", 28, 600);
        mBackSlide = new Motor(hardwareMap, "backSlide", 28, 6000);

        mFrontSlide.setInverted(false);
        mBackSlide.setInverted(true);

        mFrontSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mBackSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mFrontSlide.resetEncoder();
        mBackSlide.resetEncoder();

        mFrontSlide.setPositionCoefficient(0.05);
        mBackSlide.setPositionCoefficient(0.05);

        mFrontSlide.setPositionTolerance(10);
        mBackSlide.setPositionTolerance(10);

        mFrontSlide.setRunMode(Motor.RunMode.PositionControl);
        mBackSlide.setRunMode(Motor.RunMode.PositionControl);

        waitForStart();
        while (opModeIsActive()) {
            mFrontSlide.setTargetDistance(slideHeightTicks);
            mBackSlide.setTargetDistance(slideHeightTicks);

            mFrontSlide.set(0);
            mBackSlide.set(0);

            if(!mFrontSlide.atTargetPosition()) {
                mFrontSlide.set(0.75);
            }

            if(!mBackSlide.atTargetPosition()) {
                mBackSlide.set(0.75);
            }


            mTelemetry.addData("Fornt Slide Ticks", mFrontSlide.getCurrentPosition());
            mTelemetry.addData("Back Slide Ticks", mBackSlide.getCurrentPosition());
            mTelemetry.update();
        }

    }
}

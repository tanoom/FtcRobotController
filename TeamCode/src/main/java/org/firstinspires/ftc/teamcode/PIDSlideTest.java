package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "PIDSlideTest")
@Config
public class PIDSlideTest extends LinearOpMode {
    //private TelemetryPacket packet = new TelemetryPacket();
    private final Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int leftSlideHeightTicks = 0;
    public static int rightSlideHeightTicks = 0;
    public static double setP = 0.1;
    private Motor mFrontLeftSlide;
    private Motor mFrontRightSlide;
    private Motor mBackLeftSlide;
    private Motor mBackRightSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        mFrontLeftSlide = new Motor(hardwareMap, "frontLeftLift", 28, 6000);
        mFrontRightSlide = new Motor(hardwareMap, "frontRightLift", 28, 6000);

        mBackLeftSlide = new Motor(hardwareMap, "backLeftLift", 28, 6000);
        mBackRightSlide = new Motor(hardwareMap, "backRightLift", 28, 6000);

        mFrontLeftSlide.setInverted(false);
        mFrontRightSlide.setInverted(false);

        mBackLeftSlide.setInverted(false);
        mBackRightSlide.setInverted(true);

        mFrontLeftSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mFrontRightSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mBackLeftSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mBackRightSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mFrontLeftSlide.resetEncoder();
        mFrontRightSlide.resetEncoder();

        mBackLeftSlide.resetEncoder();
        mBackRightSlide.resetEncoder();

        mFrontLeftSlide.setPositionCoefficient(setP);
        mFrontRightSlide.setPositionCoefficient(setP);

        mBackLeftSlide.setPositionCoefficient(setP);
        mBackRightSlide.setPositionCoefficient(setP);

        mFrontLeftSlide.setPositionTolerance(10);
        mFrontRightSlide.setPositionTolerance(10);

        mBackLeftSlide.setPositionTolerance(10);
        mBackRightSlide.setPositionTolerance(10);

        mFrontLeftSlide.setRunMode(Motor.RunMode.PositionControl);
        mFrontRightSlide.setRunMode(Motor.RunMode.PositionControl);

        mBackLeftSlide.setRunMode(Motor.RunMode.PositionControl);
        mBackRightSlide.setRunMode(Motor.RunMode.PositionControl);

        waitForStart();
        while (opModeIsActive()) {
            mFrontLeftSlide.setTargetDistance(leftSlideHeightTicks);
            mFrontRightSlide.setTargetDistance(leftSlideHeightTicks);

            mBackLeftSlide.setTargetDistance(rightSlideHeightTicks);
            mBackRightSlide.setTargetDistance(rightSlideHeightTicks);

            if(!mFrontLeftSlide.atTargetPosition()) {
                mFrontLeftSlide.set(0.3);
            }
            else {
                mFrontLeftSlide.set(0);
            }

            if(!mFrontRightSlide.atTargetPosition()) {
                mFrontRightSlide.set(0.3);
            }
            else {
                mFrontRightSlide.set(0);
            }

            if(!mBackLeftSlide.atTargetPosition()) {
                mBackLeftSlide.set(0.3);
            }
            else {
                mBackLeftSlide.set(0);
            }

            if(!mBackRightSlide.atTargetPosition()) {
                mBackRightSlide.set(0.3);
            }
            else {
                mBackRightSlide.set(0);
            }



            mTelemetry.addData("FrontLeft Slide Ticks", mFrontLeftSlide.getCurrentPosition());
            mTelemetry.addData("FrontRight Slide Ticks", mFrontRightSlide.getCurrentPosition());
            mTelemetry.addData("BackLeft Slide Ticks", mBackLeftSlide.getCurrentPosition());
            mTelemetry.addData("BackRight Slide Ticks", mBackRightSlide.getCurrentPosition());
            mTelemetry.update();
        }

    }
}

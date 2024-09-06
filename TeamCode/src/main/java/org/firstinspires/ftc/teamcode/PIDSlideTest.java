package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "PIDSlideTest")
@Config
public class PIDSlideTest extends LinearOpMode {
    //private TelemetryPacket packet = new TelemetryPacket();
    private final Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int slideHeightTicks = 0;
    public static double setP = 0.1;
    private Motor mFrontSlide;
    private Motor mBackSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        mFrontSlide = new Motor(hardwareMap, "frontLeftLift", 28, 600);
        mBackSlide = new Motor(hardwareMap, "frontRightLift", 28, 6000);

        mFrontSlide.setInverted(false);
        mBackSlide.setInverted(false);

        mFrontSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mBackSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mFrontSlide.resetEncoder();
        mBackSlide.resetEncoder();

        mFrontSlide.setPositionCoefficient(setP);
        mBackSlide.setPositionCoefficient(setP);

        mFrontSlide.setPositionTolerance(10);
        mBackSlide.setPositionTolerance(10);

        mFrontSlide.setRunMode(Motor.RunMode.PositionControl);
        mBackSlide.setRunMode(Motor.RunMode.PositionControl);

        waitForStart();
        while (opModeIsActive()) {
            mFrontSlide.setTargetDistance(slideHeightTicks);
            mBackSlide.setTargetDistance(slideHeightTicks);


            if(!mFrontSlide.atTargetPosition()) {
                mFrontSlide.set(0.3);
            }
            else {
                mFrontSlide.set(0);
            }

            if(!mBackSlide.atTargetPosition()) {
                mBackSlide.set(0.3);
            }
            else {
                mBackSlide.set(0);
            }

            mTelemetry.addData("Fornt Slide Ticks", mFrontSlide.getCurrentPosition());
            mTelemetry.addData("Back Slide Ticks", mBackSlide.getCurrentPosition());
            mTelemetry.update();
        }

    }
}

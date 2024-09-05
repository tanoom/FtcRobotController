package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "PIDSecondSlideTest")
@Config
public class PIDSecondSlideTest extends LinearOpMode {
  private final MultipleTelemetry mTelemetry =
      new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
  private static double slideHeightTicks = 0;
  private DcMotorEx mFrontSlide;
  private DcMotorEx mBackSlide;

  @Override
  public void runOpMode() throws InterruptedException {
    mFrontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
    mBackSlide = hardwareMap.get(DcMotorEx.class, "backSlide");

    mFrontSlide.setDirection(DcMotorSimple.Direction.FORWARD);
    mBackSlide.setDirection(DcMotorSimple.Direction.REVERSE);

    mFrontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //        mFrontSlide.resetEncoder();
    //        mBackSlide.resetEncoder();
    //
    //        mFrontSlide.setPositionCoefficient(0.05);
    //        mBackSlide.setPositionCoefficient(0.05);
    //
    //        mFrontSlide.setPositionTolerance(10);
    //        mBackSlide.setPositionTolerance(10);
    //
    //        mFrontSlide.setRunMode(Motor.RunMode.PositionControl);
    //        mBackSlide.setRunMode(Motor.RunMode.PositionControl);

    waitForStart();
    while (opModeIsActive()) {
      //            mFrontSlide.setTargetDistance(slideHeightTicks);
      //            mBackSlide.setTargetDistance(slideHeightTicks);
      //
      //            mFrontSlide.set(0);
      //            mBackSlide.set(0);
      //
      //            if(!mFrontSlide.atTargetPosition()) {
      //                mFrontSlide.set(0.75);
      //            }
      //
      //            if(!mBackSlide.atTargetPosition()) {
      //                mBackSlide.set(0.75);
      //            }

      mTelemetry.addData("Fornt Slide Ticks", mFrontSlide.getCurrentPosition());
      mTelemetry.addData("Back Slide Ticks", mBackSlide.getCurrentPosition());
      mTelemetry.update();
    }
  }
}

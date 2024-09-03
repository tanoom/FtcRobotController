package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SuperStructure {
   public enum ReleaseLifter{
      BACK,FRONT
   }

   public static int LIFT_MIN = 0;
   public static int LIFT_LOW_Low = 1877;
   public static int LIFT_MID_Low = 2988;
   public static int LIFT_HIGH_Low = 2634;
   public static int LIFT_LOW_High = 3726;
   public static int LIFT_MID_High = 3200;
   public static int LIFT_HIGH_High = 4164;

   public static final int LIFT_TOLERANCE = 20;
   public static final double backDoor_close = 0.75,backDoor_middle = 0.5, backDoor_open = 0;

   private final DcMotorEx strafeDrive,frontLift,backLift;
   private final Servo strafe,backDoor,frontDoorLeft,frontDoorRight;
   private Runnable drive_period;
   private final LinearOpMode opMode;

   private int liftLocation;

   public SuperStructure(LinearOpMode opMode) {
      this.opMode = opMode;
      HardwareMap hardwareMap = opMode.hardwareMap;

      liftLocation = 0;

      strafeDrive = hardwareMap.get(DcMotorEx.class,"strafe_drive");
      frontLift = hardwareMap.get(DcMotorEx.class, "frontSlide");
      backLift = hardwareMap.get(DcMotorEx.class,"backSlide");

      frontLift.setDirection(DcMotorSimple.Direction.FORWARD);
      backLift.setDirection(DcMotorSimple.Direction.REVERSE);

      frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      strafe = hardwareMap.get(Servo.class,"strafe");
      backDoor = hardwareMap.get(Servo.class,"doorBack");
      frontDoorLeft = hardwareMap.get(Servo.class,"doorLeft");
      frontDoorRight = hardwareMap.get(Servo.class,"doorRight");
   }

   public void toOriginal() {
      liftLocation = 0;
      setFrontLiftPosition(LIFT_MIN, 1);
      setBackLiftPosition(LIFT_MIN,1);
   }

   public void runtimeResetLifter() {
      setLifterPower(-0.2);
      sleep_with_drive(200);
      setLifterPower(0);
      sleep_with_drive(50);
      setLifterPower(0);
      frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      setFrontLiftPosition(0,1);
      setBackLiftPosition(0,1);
   }

   public void toLowRelease(ReleaseLifter releaseLifter) {
      liftLocation = 1;
      switch (releaseLifter){
         case BACK:
            setBackLiftPosition(LIFT_LOW_Low,1);
            setFrontLiftPosition(LIFT_LOW_High,1);
            return;
         case FRONT:
            setFrontLiftPosition(LIFT_LOW_Low,1);
            setBackLiftPosition(LIFT_LOW_High,1);
      }
   }

   public void toMidRelease(ReleaseLifter releaseLifter) {
      liftLocation = 2;
      switch (releaseLifter){
         case BACK:
            setBackLiftPosition(LIFT_MID_Low,1);
            setFrontLiftPosition(LIFT_MID_High,1);
            return;
         case FRONT:
            setFrontLiftPosition(LIFT_MID_Low,1);
            setBackLiftPosition(LIFT_MID_High,1);
      }
   }

   public void toHighRelease(ReleaseLifter releaseLifter) {
      liftLocation = 3;
      switch (releaseLifter){
         case BACK:
            setBackLiftPosition(LIFT_HIGH_Low,1);
            setFrontLiftPosition(LIFT_HIGH_High,1);
            return;
         case FRONT:
            setFrontLiftPosition(LIFT_HIGH_Low,1);
            setBackLiftPosition(LIFT_HIGH_High,1);
      }
   }

   private void setFrontLiftPosition(int pos,double power){
      frontLift.setTargetPosition(pos);
      frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLift.setPower(power);
   }

   private void setBackLiftPosition(int pos,double power){
      backLift.setTargetPosition(pos);
      backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backLift.setPower(power);
   }

   private void setLifterPower(double power) {
      frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontLift.setPower(power);
      backLift.setPower(power);
      drive_period.run();
      frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontLift.setPower(power);
      backLift.setPower(power);
   }

   public double getFrontLifterPos() {
      return (double) frontLift.getCurrentPosition();
   }
   public double getBackLifterPos() {
      return (double) backLift.getCurrentPosition();
   }

   public void sleep_with_drive(double time_mm) {
      long start_time = System.currentTimeMillis();
      while (opMode.opModeIsActive() && System.currentTimeMillis() - start_time < time_mm) {
         drive_period.run();
      }
   }
}

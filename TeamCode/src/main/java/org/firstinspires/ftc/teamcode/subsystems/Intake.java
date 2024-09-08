package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake extends SubsystemBase {
    private DcMotorEx mIntakeRight;
    private DcMotorEx mIntakeLeft;
    private Servo mRollerLeft; //Continuous
    private Servo mRollerRight;
    private Servo mArmLeft; //Continuous
    private Servo mArmRight; //Continuous
    private Servo mDoorLeft; //Position
    private Servo mDoorRight; //Position

    private TouchSensor upperMag;

    private DoorState mLeftDoorState = DoorState.CLOSE;
    private DoorState mRightDoorState = DoorState.CLOSE;

    private TelemetryPacket packet = new TelemetryPacket();

    public Intake(final HardwareMap hardwareMap) {
        mIntakeLeft = hardwareMap.get(DcMotorEx.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(DcMotorEx.class,"intakeRight");
        mRollerLeft = hardwareMap.get(Servo.class,"rollerLeft"); // 0 1 reverse 0 outtake 1 intake
        mRollerRight = hardwareMap.get(Servo.class, "rollerRight"); //0 1 0 outtake 1 intake
        mArmLeft = hardwareMap.get(Servo.class, "armLeft"); //0 1 0 down 1 up
        mArmRight = hardwareMap.get(Servo.class, "armRight");//0 1 reverse 0 down 1 up
        mDoorLeft = hardwareMap.get(Servo.class, "doorLeft"); //0 0.3 1
        mDoorRight = hardwareMap.get(Servo.class, "doorRight"); //0 0.25 1 reverse

        upperMag = hardwareMap.get(TouchSensor.class, "upperMagnetic");

        mIntakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mIntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        mIntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mIntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mRollerLeft.setDirection(Servo.Direction.REVERSE);
        mRollerRight.setDirection(Servo.Direction.FORWARD);

        mArmLeft.setDirection(Servo.Direction.FORWARD);
        mArmRight.setDirection(Servo.Direction.REVERSE);

        mDoorLeft.setDirection(Servo.Direction.FORWARD);
        mDoorRight.setDirection(Servo.Direction.REVERSE);
    }

    public void setIntakePower(double power) {
        mIntakeLeft.setPower(power);
        mIntakeRight.setPower(power);
    }

    public void setRollerPower(double power) {
        mRollerLeft.setPosition(power);
        mRollerRight.setPosition(power);
    }

    public void setArmPower(double power) {
        mArmLeft.setPosition(power);
        mArmRight.setPosition(power);
    }

    public void riseArm() {
        if(!upperMag.isPressed()) {
            mArmLeft.setPosition(0.8);
            mArmRight.setPosition(0.8);
        }
        else {
            mArmLeft.setPosition(0.5);
            mArmRight.setPosition(0.5);
        }
    }

    public boolean getUpperMagPressed() {
        return upperMag.isPressed();
    }

    public void switchLeftDoorState() {
        switch (mLeftDoorState){
            case OPEN:
                mDoorLeft.setPosition(1);
                mLeftDoorState = DoorState.CLOSE;
                break;
            case CLOSE:
                mDoorLeft.setPosition(0.3);
                mLeftDoorState = DoorState.OPEN;
        }
    }

    public void switchRightDoorState() {
        switch (mRightDoorState){
            case OPEN:
                mDoorRight.setPosition(1);
                mRightDoorState = DoorState.CLOSE;
                break;
            case CLOSE:
                mDoorRight.setPosition(0.25);
                mRightDoorState = DoorState.OPEN;
        }
    }


    public enum ArmState {
        RISING, FALLING, HOLDING;
    }

    public enum DoorState {
        OPEN, CLOSE;
    }




    @Override
    public void periodic() {
        packet.put("Is Upper Trigger", upperMag.isPressed());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

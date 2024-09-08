package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
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

public class Intake extends SubsystemBase {
    private DcMotorEx mIntakeRight;
    private DcMotorEx mIntakeLeft;
    private Servo mRoller; //Continuous
    private Servo mArmLeft; //Continuous
    private Servo mArmRight; //Continuous
    private Servo mDoorLeft; //Position
    private Servo mDoorRight; //Position

    public Intake(final HardwareMap hardwareMap) {
        mIntakeLeft = hardwareMap.get(DcMotorEx.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(DcMotorEx.class,"intakeRight");
        mRoller = hardwareMap.get(Servo.class,"roller");
        mArmLeft = hardwareMap.get(Servo.class, "armLeft");
        mArmRight = hardwareMap.get(Servo.class, "armRight");
        mDoorLeft = hardwareMap.get(Servo.class, "doorLeft");
        mDoorRight = hardwareMap.get(Servo.class, "doorRight");

        mIntakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mIntakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        mRoller.setDirection(Servo.Direction.FORWARD);
        mArmLeft.setDirection(Servo.Direction.FORWARD);
        mArmRight.setDirection(Servo.Direction.FORWARD);
        mDoorLeft.setDirection(Servo.Direction.FORWARD);
        mDoorRight.setDirection(Servo.Direction.FORWARD);
    }

    public void setIntakePower(double power) {
        mIntakeLeft.setPower(power);
        mIntakeRight.setPower(power);
    }

    public void setRollerPower(double power) {
        mRoller.setPosition(power);
    }

    public void setArmPower(double power) {
        mArmLeft.setPosition(power);
        mArmRight.setPosition(power);
    }



    @Override
    public void periodic() {

    }
}

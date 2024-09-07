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
    private DcMotorEx mInRight;
    private DcMotorEx mInLeft;
    private Servo take;

    public Intake(final HardwareMap hardwareMap) {
        mInLeft = hardwareMap.get(DcMotorEx.class,"intakeLeft");
        mInRight = hardwareMap.get(DcMotorEx.class,"intakeRight");
        take = hardwareMap.get(Servo.class,"take");

        mInLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mInRight.setDirection(DcMotorSimple.Direction.FORWARD);
        take.setDirection(Servo.Direction.FORWARD);
    }

    public void activateIn() {
        mInLeft.setPower(0.8);
        mInLeft.setPower(0.8);
        FtcDashboard.getInstance().getTelemetry().addData("Yes", "In is triggered");
    }

    public void stopIntake() {
        mInLeft.setPower(0);
        mInLeft.setPower(0);
        FtcDashboard.getInstance().getTelemetry().addData("Yes", "Intake is stopped");
    }
}

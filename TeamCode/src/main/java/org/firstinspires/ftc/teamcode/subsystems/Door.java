package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Door extends SubsystemBase {
    private Servo backDoor;
    private Servo frontDoorLeft;
    private Servo frontDoorRight;

    public Door(final HardwareMap hardwareMap) {
        backDoor = hardwareMap.get(Servo.class,"doorBack");
        frontDoorLeft = hardwareMap.get(Servo.class,"doorLeft");
        frontDoorRight = hardwareMap.get(Servo.class,"doorRight");

        backDoor.setDirection(Servo.Direction.FORWARD);
        frontDoorLeft.setDirection(Servo.Direction.FORWARD);
        frontDoorRight.setDirection(Servo.Direction.REVERSE);
    }

    public void openBackDoor() {
        backDoor.setPosition(0.7);
    }

    public void closeBackDoor() {
        backDoor.setPosition(0);
    }

    public void openLeftDoor(double servoPower) {
        frontDoorLeft.setPosition(servoPower);
        FtcDashboard.getInstance().getTelemetry().addData("Yes", "Left is Triggered");
    }

    public void openRightDoor(double servoPower) {
        frontDoorRight.setPosition(servoPower);
        FtcDashboard.getInstance().getTelemetry().addData("Yes", "Right is Triggered");
    }
}

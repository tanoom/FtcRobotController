package org.firstinspires.ftc.teamcode.subsystems;

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

    public void openLeftDoor() {
        frontDoorLeft.setPosition(0.7);
    }

    public void openRightDoor() {
        frontDoorRight.setPosition(0.7);
    }
}

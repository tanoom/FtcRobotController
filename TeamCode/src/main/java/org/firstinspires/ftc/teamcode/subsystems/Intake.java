package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake extends SubsystemBase {
    private Motor mIntakeRight;
    private Motor mIntakeLeft;
    private Servo mRollerLeft; //Continuous
    private Servo mRollerRight;
    private Servo mArmLeft; //Continuous
    private Servo mArmRight; //Continuous
    private Servo mDoorLeft; //Position
    private Servo mDoorRight; //Position

    private PIDFController leftController = new PIDController(0.04, 0, 0);
    private PIDFController rightController = new PIDController(0.04, 0, 0);

    private ColorSensor mLeftColorSensor;
    //private ColorSensor mRightColorSensor;

    private IntakeState intakeState = IntakeState.STOW;

    private TouchSensor upperMag;

    private DoorState mLeftDoorState = DoorState.CLOSE;
    private DoorState mRightDoorState = DoorState.CLOSE;

    private TelemetryPacket packet = new TelemetryPacket();

    public Intake(final HardwareMap hardwareMap) {
        mIntakeLeft = new Motor(hardwareMap, "intakeLeft");
        mIntakeRight = new Motor(hardwareMap, "intakeRight");
        mRollerLeft = hardwareMap.get(Servo.class,"rollerLeft"); // 0 1 reverse 0 outtake 1 intake
        mRollerRight = hardwareMap.get(Servo.class, "rollerRight"); //0 1 0 outtake 1 intake
        mArmLeft = hardwareMap.get(Servo.class, "armLeft"); //0 1 0 down 1 up
        mArmRight = hardwareMap.get(Servo.class, "armRight");//0 1 reverse 0 down 1 up
        mDoorLeft = hardwareMap.get(Servo.class, "doorLeft"); //0 0.3 1
        mDoorRight = hardwareMap.get(Servo.class, "doorRight"); //0 0.25 1 reverse
        mLeftColorSensor = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        mRightColorSensor = hardwareMap.get(ColorSensor.class, "colorSensorRight");

        upperMag = hardwareMap.get(TouchSensor.class, "upperMagnetic");

        mIntakeLeft.setInverted(false);
        mIntakeRight.setInverted(true);

        mIntakeLeft.stopAndResetEncoder();
        mIntakeRight.stopAndResetEncoder();

        mIntakeLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mIntakeRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mIntakeLeft.setRunMode(Motor.RunMode.RawPower);
        mIntakeRight.setRunMode(Motor.RunMode.RawPower);

        mRollerLeft.setDirection(Servo.Direction.REVERSE);
        mRollerRight.setDirection(Servo.Direction.FORWARD);

        mArmLeft.setDirection(Servo.Direction.FORWARD);
        mArmRight.setDirection(Servo.Direction.REVERSE);

        mDoorLeft.setDirection(Servo.Direction.FORWARD);
        mDoorRight.setDirection(Servo.Direction.REVERSE);

        mLeftColorSensor.enableLed(true);
        //mRightColorSensor.enableLed(true);

        resetController();
        setIntakePosition(IntakeState.STOW);
    }

    public void setIntakePower(double power) {
        mIntakeLeft.set(power);
        mIntakeRight.set(power);
    }

    public void resetController() {
        leftController.reset();
        rightController.reset();
    }

    public void setIntakePosition(IntakeState intakeState) {
        this.intakeState = intakeState;
        leftController.setSetPoint(intakeState.position);
        rightController.setSetPoint(intakeState.position);
    }

    public void moveIntakeToPosition() {

        boolean leftColorBallDetected = mLeftColorSensor.red() >= 1000
                || mLeftColorSensor.green() >= 1000
                || mLeftColorSensor.blue() >= 1000;
//        boolean rightColorBallDetected = mRightColorSensor.red() >= 1000
//                || mRightColorSensor.green() >= 1000
//                || mRightColorSensor.blue() >= 1000;

        if((leftColorBallDetected) && intakeState == IntakeState.PUSH)
            setIntakePosition(IntakeState.GRAB);

        if(!leftController.atSetPoint()) {
            mIntakeLeft.set(leftController.calculate(mIntakeLeft.encoder.getDistance()));
        }
        else {
            mIntakeLeft.set(0);
        }

        if(!rightController.atSetPoint()) {
            mIntakeRight.set(rightController.calculate(mIntakeRight.encoder.getDistance()));
        }
        else {
            mIntakeRight.set(0);
        }
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

    public enum IntakeState{
        STOW(3),
        PUSH(100),
        GRAB(165);
        private final double position;
        IntakeState(double position) {
            this.position = position;
        }
    }




    @Override
    public void periodic() {
        moveIntakeToPosition();

        packet.put("Is Upper Trigger", upperMag.isPressed());
        packet.put("Color Red", mLeftColorSensor.red());
        packet.put("Color Blue", mLeftColorSensor.blue());
        packet.put("Color Green", mLeftColorSensor.green());
        packet.put("Left Setpoint", leftController.getSetPoint());
        packet.put("Right Setpoint", rightController.getSetPoint());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

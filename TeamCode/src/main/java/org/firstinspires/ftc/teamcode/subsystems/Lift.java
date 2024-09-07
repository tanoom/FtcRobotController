package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {

    private DcMotorEx mFrontLeftSlide;
    private DcMotorEx mBackLeftSlide;
    private DcMotorEx mFrontRightSlide;
    private DcMotorEx mBackRightSlide;

    private LiftState mFrontLiftState = LiftState.IDLE;
    private LiftState mBackLiftState = LiftState.IDLE;

    private ReleaseDirection releaseDirection = ReleaseDirection.IDLE;
    private ReleaseLevel releaseLevel = ReleaseLevel.ORIGIN;

    private TelemetryPacket packet = new TelemetryPacket();

    //private final ElevatorFeedforward ff = new ElevatorFeedforward(0, 0, 0, 0);

    public Lift(final HardwareMap hardwareMap) {
        mFrontLeftSlide = hardwareMap.get(DcMotorEx.class, "frontLeftLift");
        mBackLeftSlide = hardwareMap.get(DcMotorEx.class, "backLeftLift");
        mFrontRightSlide = hardwareMap.get(DcMotorEx.class, "frontRightLift");
        mBackRightSlide = hardwareMap.get(DcMotorEx.class, "backRightLift");

        mFrontLeftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        mBackLeftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        mFrontRightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        mBackRightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        mFrontLeftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackLeftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFrontRightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackRightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAndResetEncoder();
        mFrontLeftSlide.setTargetPosition(0);
        mBackLeftSlide.setTargetPosition(0);
        mFrontRightSlide.setTargetPosition(0);
        mBackRightSlide.setTargetPosition(0);
        setPositionMode();
    }

    public void setReleaseDirection(ReleaseDirection direction) {
        releaseDirection = direction;
        //updateCurrentState();
    }

    public void setReleaseLevel(ReleaseLevel level) {
        releaseLevel = level;
        updateCurrentState();
    }

    public void liftUp() {
        switch (releaseLevel) {
            case ORIGIN:
                releaseLevel = ReleaseLevel.LOW;
                packet.put("yes", "You trigger the ORIGIN");
                return;
            case LOW:
                releaseLevel = ReleaseLevel.MID;
                packet.put("yes", "You trigger the LOW");
                return;
            case MID:
                releaseLevel = ReleaseLevel.HIGH;
                packet.put("yes", "You trigger the MID");
                return;
            case HIGH:
                releaseLevel = ReleaseLevel.HIGH;
                packet.put("yes", "You trigger the HIGH");
        }
        //updateCurrentState();
    }

    public void fallDown() {
        switch (releaseLevel) {
            case ORIGIN:
                releaseLevel = ReleaseLevel.ORIGIN;
                packet.put("yes", "You trigger the ORIGIN");
                return;
            case LOW:
                releaseLevel = ReleaseLevel.ORIGIN;
                packet.put("yes", "You trigger the LOW");
                return;
            case MID:
                releaseLevel = ReleaseLevel.LOW;
                packet.put("yes", "You trigger the MID");
                return;
            case HIGH:
                releaseLevel = ReleaseLevel.MID;
                packet.put("yes", "You trigger the HIGH");
        }
        //updateCurrentState();
    }

    public void updateCurrentState() {
        switch (releaseLevel) {
            case ORIGIN:
                setOriginal();
                return;
            case LOW:
                setLowRelease(releaseDirection);
                return;
            case MID:
                setMidRelease(releaseDirection);
                return;
            case HIGH:
                setHighRelease(releaseDirection);
        }
        //TODO Need to consider whether to add this
        setPositionMode();
    }

    public void setLowRelease(ReleaseDirection direction) {
        switch (direction) {
            case FRONT:
                mFrontLiftState = LiftState.LOW_GOAL_LOW;
                mBackLiftState = LiftState.LOW_GOAL_HIGH;
                return;
            case BACK:
                mFrontLiftState = LiftState.LOW_GOAL_HIGH;
                mBackLiftState = LiftState.LOW_GOAL_LOW;
                return;
            case IDLE:
                mFrontLiftState = LiftState.LOW_GOAL_LOW;
                mBackLiftState = LiftState.LOW_GOAL_LOW;
        }
    }

    public void setMidRelease(ReleaseDirection direction) {
        switch (direction) {
            case FRONT:
                mFrontLiftState = LiftState.MID_GOAL_LOW;
                mBackLiftState = LiftState.MID_GOAL_HIGH;
                return;
            case BACK:
                mFrontLiftState = LiftState.MID_GOAL_HIGH;
                mBackLiftState = LiftState.MID_GOAL_LOW;
                return;
            case IDLE:
                mFrontLiftState = LiftState.MID_GOAL_LOW;
                mBackLiftState = LiftState.MID_GOAL_LOW;
        }
    }

    public void setHighRelease(ReleaseDirection direction) {
        switch (direction) {
            case FRONT:
                mFrontLiftState = LiftState.HIGH_GOAL_LOW;
                mBackLiftState = LiftState.HIGH_GOAL_HIGH;
                return;
            case BACK:
                mFrontLiftState = LiftState.HIGH_GOAL_HIGH;
                mBackLiftState = LiftState.HIGH_GOAL_LOW;
                return;
            case IDLE:
                mFrontLiftState = LiftState.HIGH_GOAL_LOW;
                mBackLiftState = LiftState.HIGH_GOAL_LOW;
        }
    }

    public void setOriginal() {
        mFrontLiftState = LiftState.ORIGINAL;
        mBackLiftState = LiftState.ORIGINAL;
    }

    public void stopMotor() {
        mFrontLiftState = LiftState.IDLE;
        mBackLiftState = LiftState.IDLE;
    }

    public void setPositionMode() {
        mFrontLeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackLeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mFrontRightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackRightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopAndResetEncoder() {
        mFrontLeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackLeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFrontRightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackRightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double frontPower, double backPower) {
        mFrontLeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBackLeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFrontRightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBackRightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(mFrontLeftSlide.getCurrentPosition() < 1291) {
            mFrontLeftSlide.setPower(frontPower);
        }

        if(mBackLeftSlide.getCurrentPosition() < 1291) {
            mBackLeftSlide.setPower(backPower);
        }
        if(mFrontRightSlide.getCurrentPosition() < 1291) {
            mFrontRightSlide.setPower(frontPower);
        }

        if(mBackRightSlide.getCurrentPosition() < 1291) {
            mBackRightSlide.setPower(backPower);
        }
    }



    public enum LiftState{
        IDLE(0, 0),
        ORIGINAL(0, 1),
        LOW_GOAL_LOW(469, 1),
        LOW_GOAL_HIGH(747, 1),
        MID_GOAL_LOW(659, 1),
        MID_GOAL_HIGH(931, 1),
        HIGH_GOAL_LOW(800, 1),
        HIGH_GOAL_HIGH(1291, 1);

        private final int height;
        private final int power;
        private LiftState(int height, int power) {
            this.height = height;
            this.power = power;
        }
    }

    public enum ReleaseDirection {
        FRONT, BACK, IDLE;

        @Override
        public String toString() {
            switch (this) {
                case FRONT:
                    return "FRONT";
                case BACK:
                    return "BACK";
                case IDLE:
                    return "IDLE";
                default:
                    return "WRONG STATE";
            }
        }
    }

    public enum ReleaseLevel {
        ORIGIN, LOW, MID, HIGH;

        @Override
        public String toString() {
            switch (this) {
                case ORIGIN:
                    return "ORIGIN";
                case LOW:
                    return "LOW";
                case MID:
                    return "MID";
                case HIGH:
                    return "HIGH";
                default:
                    return "WRONG STATE";
            }
        }
    }

    @Override
    public void periodic(){
        updateCurrentState();

//        mFrontSlide.setTargetPosition(mFrontLiftState.height);
//        mFrontSlide.setPower(mFrontLiftState.power);
//
//        mBackSlide.setTargetPosition(mBackLiftState.height);
//        mBackSlide.setPower(mBackLiftState.power);


        packet.put("Direction", releaseDirection);
        packet.put("Level", releaseLevel);
        packet.put("Front Height", mFrontLiftState.height);
        packet.put("Back Height", mBackLiftState.height);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

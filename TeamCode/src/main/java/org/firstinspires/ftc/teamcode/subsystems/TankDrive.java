package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.util.MathUtils;

public class TankDrive extends SubsystemBase {
    private final DcMotorEx leftDrive;
    private final DcMotorEx rightDrive;
    private final DcMotorEx strafeDrive;


    public TankDrive (final HardwareMap hardwareMap) {
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        strafeDrive = hardwareMap.get(DcMotorEx.class,"strafe_drive");

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        strafeDrive.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        // Normalize wheel powers to be less than 1.0
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        // Send powers to the wheels.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }


    @Override
    public void periodic() {

    }
}

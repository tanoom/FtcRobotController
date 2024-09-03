package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.TankDrive;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {
    private final TankDrive tankDrive;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier turnSupplier;

    public TankDriveCommand(TankDrive tankDrive, DoubleSupplier drive, DoubleSupplier turn) {
        this.tankDrive = tankDrive;
        this.driveSupplier = drive;
        this.turnSupplier = turn;

        addRequirements(tankDrive);
    }

    @Override
    public void execute() {
        tankDrive.moveRobot(
                driveSupplier.getAsDouble(),
                turnSupplier.getAsDouble()
        );
    }
}

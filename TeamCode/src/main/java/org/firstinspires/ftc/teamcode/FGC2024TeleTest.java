package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Door;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TankDrive;

@TeleOp(name = "FGC 2024 CommandOP")
public class FGC2024TeleTest extends CommandOpMode {

    private TankDrive tankDrive;
    private Lift lift;
    private Door door;
    private GamepadEx gamepadEx;
    //private GamepadEx gamepad2;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        //Subsystems Initialization
        tankDrive = new TankDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        door = new Door(hardwareMap);

        gamepadEx = new GamepadEx(gamepad1);

        //Default Commands Binding
        tankDrive.setDefaultCommand(new TankDriveCommand(tankDrive,
            () -> -gamepadEx.getLeftY(), () -> gamepadEx.getRightX()));

        //Buttons Binding
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> lift.setReleaseDirection(Lift.ReleaseDirection.FRONT))
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> lift.setReleaseDirection(Lift.ReleaseDirection.BACK))
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> lift.liftUp())
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> lift.fallDown())
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> lift.stopMotor())
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> door.openLeftDoor())
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> door.openRightDoor())
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> door.openBackDoor())
        );
    }



}

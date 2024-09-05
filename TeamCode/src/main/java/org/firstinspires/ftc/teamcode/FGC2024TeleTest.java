package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.LiftOpenLoopCommand;
import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.common.util.FunctionalButton;
import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.Door;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TankDrive;

@TeleOp(name = "FGC 2024 CommandOP")
public class FGC2024TeleTest extends CommandOpMode {
    private TriggerReader triggerReader;
    private SlewRateLimiter driverLimiter;
    private SlewRateLimiter turnLimiter;

    private TankDrive tankDrive;
    private Lift lift;
    private Door door;
    private GamepadEx gamepadEx1, gamepadEx2;
    //private GamepadEx gamepad2;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        //Subsystems Initialization
        tankDrive = new TankDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        door = new Door(hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        driverLimiter = new SlewRateLimiter(3);
        turnLimiter = new SlewRateLimiter(3);


        TriggerReader frontLiftUp = new TriggerReader(
                gamepadEx1, GamepadKeys.Trigger.LEFT_TRIGGER
        );

        TriggerReader backLiftUp = new TriggerReader(
                gamepadEx1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

    // Default Commands Binding
    //        tankDrive.setDefaultCommand(new TankDriveCommand(tankDrive,
    //            () -> -gamepadEx2.getLeftY(), () -> gamepadEx2.getRightX()));

    tankDrive.setDefaultCommand(
        new TankDriveCommand(
            tankDrive,
            () -> -driverLimiter.calculate(gamepadEx2.getLeftY()) ,
            () -> gamepadEx2.getRightX(),
            () -> gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 ));

        lift.setDefaultCommand(new LiftOpenLoopCommand(
                lift, () -> gamepadEx1.getLeftY(), () -> -gamepadEx1.getRightY()
        ));

        //Buttons Binding
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                new InstantCommand(() -> lift.setReleaseDirection(Lift.ReleaseDirection.FRONT))
//        );

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                new InstantCommand(() -> lift.setReleaseDirection(Lift.ReleaseDirection.BACK))
//        );
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new InstantCommand(() -> lift.liftUp())
//        );
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new InstantCommand(() -> lift.fallDown())
//        );

//        new FunctionalButton(() -> getDPADAngle(gamepadEx1) == 90).whenPressed(
//                new InstantCommand(() -> lift.liftUp())
//        );
//
//        new FunctionalButton(() -> getDPADAngle(gamepadEx1) == 270).whenPressed(
//                new InstantCommand(() -> lift.fallDown())
//        );
//
//        new FunctionalButton(() -> getDPADAngle(gamepadEx1) == 0).whenPressed(
//                new InstantCommand(() -> lift.setReleaseDirection(Lift.ReleaseDirection.BACK))
//        );
//
//        new FunctionalButton(() -> getDPADAngle(gamepadEx1) == 180).whenPressed(
//                new InstantCommand(() -> lift.setReleaseDirection(Lift.ReleaseDirection.FRONT))
//        );
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new InstantCommand(() -> lift.stopMotor())
//        );


        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> door.openLeftDoor(1))
        ).whenReleased(new InstantCommand(() -> door.openLeftDoor(0.5)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> door.openRightDoor(1))
        ).whenReleased(new InstantCommand(() -> door.openRightDoor(0.5)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> door.openLeftDoor(0))
        ).whenReleased(new InstantCommand(() -> door.openLeftDoor(0.5)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> door.openRightDoor(0))
        ).whenReleased(new InstantCommand(() -> door.openRightDoor(0.5)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> door.openBackDoor())
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> door.closeBackDoor())
        );



    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();


    }



    public int getDPADAngle(GamepadEx gamepad) {
        if(gamepad == null) return -1;
        boolean[] buttonArray = {
                gamepad.getButton(GamepadKeys.Button.DPAD_LEFT),
                gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT),
                gamepad.getButton(GamepadKeys.Button.DPAD_UP),
                gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)
        };
        if(buttonArray[0] && !buttonArray[1] && !buttonArray[2] && !buttonArray[3]) {
            return 180;
        }
        if(!buttonArray[0] && buttonArray[1] && buttonArray[2] && buttonArray[3]) {
            return 0;
        }
        if(!buttonArray[0] && !buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
            return 90;
        }
        if(!buttonArray[0] && !buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
            return 270;
        }
        if(buttonArray[0] && !buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
            return 135;
        }
        if(buttonArray[0] && !buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
            return 225;
        }
        if(!buttonArray[0] && buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
            return 45;
        }
        if(buttonArray[0] && buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
            return 315;
        }

        return -1;
    }



}

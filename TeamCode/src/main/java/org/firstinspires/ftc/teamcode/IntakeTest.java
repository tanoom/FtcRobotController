package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {
    private Motor intakeLeft;
    private Motor intakeRight;
    private Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {

        intakeLeft = new Motor(hardwareMap, "intakeLeft");
        intakeRight = new Motor(hardwareMap, "intakeRight");
        intakeLeft.resetEncoder();
        intakeRight.resetEncoder();

        waitForStart();

        while (opModeIsActive()) {
            mTelemetry.addData("leftPos", intakeLeft.encoder.getDistance());
            mTelemetry.addData("rightPos", intakeRight.encoder.getDistance());
            mTelemetry.update();
        }

    }
}

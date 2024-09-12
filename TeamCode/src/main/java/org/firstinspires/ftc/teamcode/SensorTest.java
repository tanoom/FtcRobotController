package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "Seonsor")
public class SensorTest extends LinearOpMode {
    //private TouchSensor magnetic;
    //private DistanceSensor distanceSensor;
    private Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private TelemetryPacket packet = new TelemetryPacket();
    private ColorSensor colorSensor; //Either of the color is greater than 1000, then it is on the intake position



    @Override
    public void runOpMode() throws InterruptedException {
        //touch = hardwareMap.get(TouchSensor.class, "magneticSensor");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensorRight");

        waitForStart();
        while (opModeIsActive()) {
            mTelemetry.addData("Red Value", colorSensor.red());
            mTelemetry.addData("Green Value", colorSensor.green());
            mTelemetry.addData("Blue Value", colorSensor.blue());
//            mTelemetry.addData("Is Magnetic Triggerd", touch.isPressed());
//            mTelemetry.addData("Value of Magnetic", touch.getValue());
//            mTelemetry.addData("Distance of Sensor", distanceSensor.getDistance(DistanceUnit.CM));
            mTelemetry.update();
        }
    }
}

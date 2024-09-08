package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "servo test")
@Config
public class ServoTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static boolean read_only = false;
    public static boolean reverse = true;
    public static double servo_pos1 = 1;

    public static String servo_name1 = "armRight";
    public static String servo_name2 = "armLeft";
    private Servo servo0 = null;
    private Servo servo1 = null;
    private TouchSensor upperMagnetic;

    private boolean prevVal = false;
    private boolean shouldStop = false;

    @Override
    public void runOpMode() {

        servo0 = hardwareMap.get(Servo.class, servo_name1);
        servo1 = hardwareMap.get(Servo.class, servo_name2);
        upperMagnetic = hardwareMap.get(TouchSensor.class, "upperMagnetic");
        if (reverse){
            servo0.setDirection(Servo.Direction.REVERSE);
        }
        waitForStart();
        while (opModeIsActive()) {

            if (!read_only && !upperMagnetic.isPressed()) {
                servo0.setPosition(servo_pos1);
                servo1.setPosition(servo_pos1);
                //servo1.setPosition(servo_pos1);
//                servo1.setPosition(servo_pos2);
                telemetry_M.addData(servo_name1, servo0.getPosition());
//                telemetry_M.addData("rightfront", servo1.getPosition());

            }
            else {
                servo0.setPosition(0.5);
                servo1.setPosition(0.5);
            }

            telemetry_M.addData("Magnetic Triggered", upperMagnetic.isPressed());
            telemetry_M.update();
        }
    }
}

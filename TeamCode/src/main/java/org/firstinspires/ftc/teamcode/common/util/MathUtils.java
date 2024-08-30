package org.firstinspires.ftc.teamcode.common.util;

public class MathUtils {

    public static double toSlope(Number angle) {
        return Math.atan(Math.toRadians(angle.doubleValue() - 90));
    }

}

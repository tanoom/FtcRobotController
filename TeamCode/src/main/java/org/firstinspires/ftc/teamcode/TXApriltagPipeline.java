package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class TXApriltagPipeline extends AprilTagProcessor {

    @Override
    public void setDecimation(float decimation) {

    }

    @Override
    public void setPoseSolver(PoseSolver poseSolver) {

    }

    @Override
    public int getPerTagAvgPoseSolveTime() {
        return 0;
    }

    @Override
    public ArrayList<AprilTagDetection> getDetections() {
        return null;
    }

    @Override
    public ArrayList<AprilTagDetection> getFreshDetections() {
        return null;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}

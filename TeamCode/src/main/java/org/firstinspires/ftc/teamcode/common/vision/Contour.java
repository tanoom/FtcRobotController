package org.firstinspires.ftc.teamcode.common.vision;

import org.firstinspires.ftc.teamcode.common.util.MathUtil;
import org.opencv.core.CvType;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Rect2d;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Arrays;
import java.util.Collection;

import javax.annotation.Nullable;

public class Contour implements Releasable{
    public final MatOfPoint mat;
    private Double area = Double.NaN;
    private Double perimeter = Double.NaN;
    private MatOfPoint2f mat2f = null;
    private RotatedRect minAreaRect = null;
    private Rect boundingRect = null;
    private Moments moments = null;

    private MatOfPoint2f convexHull = null;
    private MatOfPoint2f approxPolyDp = null;

    public Contour(MatOfPoint mat) {
        this.mat = mat;
    }

    public Contour(Rect2d box) {
        // no easy way to convert a Rect2d to Mat, diy it. Order is tl tr br bl
        this.mat =
                new MatOfPoint(
                        box.tl(),
                        new Point(box.x + box.width, box.y),
                        box.br(),
                        new Point(box.x, box.y + box.height));
    }

    public MatOfPoint2f getMat2f() {
        if (mat2f == null) {
            mat2f = new MatOfPoint2f(mat.toArray());
            mat.convertTo(mat2f, CvType.CV_32F);
        }
        return mat2f;
    }

    public MatOfPoint2f getConvexHull() {
        if (this.convexHull == null) {
            MatOfInt ints = new MatOfInt();
            Imgproc.convexHull(mat, ints);
            this.convexHull = Contour.convertIndexesToPoints(mat, ints);
            ints.release();
        }
        return convexHull;
    }

    public MatOfPoint2f getApproxPolyDp(double epsilon, boolean closed) {
        if (this.approxPolyDp == null) {
            approxPolyDp = new MatOfPoint2f();
            Imgproc.approxPolyDP(getConvexHull(), approxPolyDp, epsilon, closed);
        }
        return approxPolyDp;
    }

    @Nullable
    public MatOfPoint2f getApproxPolyDp() {
        return this.approxPolyDp;
    }

    public double getArea() {
        if (Double.isNaN(area)) {
            area = Imgproc.contourArea(mat);
        }
        return area;
    }

    public double getPerimeter() {
        if (Double.isNaN(perimeter)) {
            perimeter = Imgproc.arcLength(getMat2f(), true);
        }
        return perimeter;
    }

    public RotatedRect getMinAreaRect() {
        if (minAreaRect == null) {
            minAreaRect = Imgproc.minAreaRect(getMat2f());
        }
        return minAreaRect;
    }

    public Rect getBoundingRect() {
        if (boundingRect == null) {
            boundingRect = Imgproc.boundingRect(mat);
        }
        return boundingRect;
    }

    public Moments getMoments() {
        if (moments == null) {
            moments = Imgproc.moments(mat);
        }
        return moments;
    }

    public boolean isEmpty() {
        return mat.empty();
    }

    public Point getCenterPoint() {
        return getMinAreaRect().center;
    }

    public boolean isIntersecting(
            Contour secondContour, ContourIntersectionDirection intersectionDirection) {
        boolean isIntersecting = false;

        if (intersectionDirection == ContourIntersectionDirection.None) {
            isIntersecting = true;
        } else {
            try {
                MatOfPoint2f intersectMatA = new MatOfPoint2f();
                MatOfPoint2f intersectMatB = new MatOfPoint2f();

                mat.convertTo(intersectMatA, CvType.CV_32F);
                secondContour.mat.convertTo(intersectMatB, CvType.CV_32F);

                RotatedRect a = Imgproc.fitEllipse(intersectMatA);
                RotatedRect b = Imgproc.fitEllipse(intersectMatB);
                double mA = MathUtil.toSlope(a.angle);
                double mB = MathUtil.toSlope(b.angle);
                double x0A = a.center.x;
                double y0A = a.center.y;
                double x0B = b.center.x;
                double y0B = b.center.y;
                double intersectionX = ((mA * x0A) - y0A - (mB * x0B) + y0B) / (mA - mB);
                double intersectionY = (mA * (intersectionX - x0A)) + y0A;
                double massX = (x0A + x0B) / 2;
                double massY = (y0A + y0B) / 2;
                switch (intersectionDirection) {
                    case Up:
                        if (intersectionY < massY) isIntersecting = true;
                        break;
                    case Down:
                        if (intersectionY > massY) isIntersecting = true;
                        break;
                    case Left:
                        if (intersectionX < massX) isIntersecting = true;
                        break;
                    case Right:
                        if (intersectionX > massX) isIntersecting = true;
                        break;
                }
                intersectMatA.release();
                intersectMatB.release();
            } catch (Exception e) {
                // defaults to false
            }
        }

        return isIntersecting;
    }

    // TODO: refactor to do "infinite" contours ???????
    public static Contour groupContoursByIntersection(
            Contour firstContour, Contour secondContour, ContourIntersectionDirection intersection) {
        if (areIntersecting(firstContour, secondContour, intersection)) {
            return combineContours(firstContour, secondContour);
        } else {
            return null;
        }
    }

    public static boolean areIntersecting(
            Contour firstContour,
            Contour secondContour,
            ContourIntersectionDirection intersectionDirection) {
        return firstContour.isIntersecting(secondContour, intersectionDirection)
                || secondContour.isIntersecting(firstContour, intersectionDirection);
    }

    public static Contour combineContours(Contour... contours) {
        return combineContourList(Arrays.asList(contours));
    }

    public static Contour combineContourList(Collection<Contour> contours) {
        MatOfPoint points = new MatOfPoint();

        for (Contour contour : contours) {
            points.push_back(contour.mat);
        }

        Contour finalContour = new Contour(points);

        boolean contourEmpty = finalContour.isEmpty();
        return contourEmpty ? null : finalContour;
    }

    @Override
    public void release() {
        if (mat != null) mat.release();
        if (mat2f != null) mat2f.release();
        if (convexHull != null) convexHull.release();
        if (approxPolyDp != null) approxPolyDp.release();
    }

    public static MatOfPoint2f convertIndexesToPoints(MatOfPoint contour, MatOfInt indexes) {
        int[] arrIndex = indexes.toArray();
        Point[] arrContour = contour.toArray();
        Point[] arrPoints = new Point[arrIndex.length];

        for (int i = 0; i < arrIndex.length; i++) {
            arrPoints[i] = arrContour[arrIndex[i]];
        }

        MatOfPoint2f hull = new MatOfPoint2f();
        hull.fromArray(arrPoints);
        return hull;
    }

    public enum ContourIntersectionDirection {
        None,
        Up,
        Down,
        Left,
        Right
    }
}

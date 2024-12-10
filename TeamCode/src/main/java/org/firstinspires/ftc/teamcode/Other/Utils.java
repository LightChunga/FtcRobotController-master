package org.firstinspires.ftc.teamcode.Other;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.Consumer;

public class Utils {

    static public DcMotor leftDrive = null;
    static public DcMotor rightDrive = null;
    static public DcMotor leftDrive_fata = null;
    static public DcMotor rightDrive_fata = null;
    static public DcMotor motorBrat = null;
    static public Servo gheara_s = null;
    static public Servo gheara_d = null;
    static public Servo spin = null;
    static public OpenCvCamera webc;
    static public PrompDeterminationPipeline pipeline;
    static public DistanceSensor dist_left = null, dist_right = null;
    static public IMU imm = null;
    static public SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    static final int inf = 9999999;

    static public PrompDeterminationPipeline.PrompPosition position = PrompDeterminationPipeline.PrompPosition.LEFT;

    static public ColorSensor pix2, pix1;

    ///deprecated, use RoadRunner turn
    public void spin360(int val) {

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - val);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + val);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getTargetPosition() - val);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getTargetPosition() + val);

        leftDrive.setPower(0.4);
        leftDrive_fata.setPower(0.4);
        rightDrive.setPower(0.4);
        rightDrive_fata.setPower(0.4);

        while (leftDrive.isBusy()) {

        }

        leftDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive.setPower(0);
        rightDrive_fata.setPower(0);
    }

    public void line(int val) {

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - val);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - val);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition() - val);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition() - val);

        leftDrive.setPower(0.4);
        leftDrive_fata.setPower(0.4);
        rightDrive.setPower(0.4);
        rightDrive_fata.setPower(0.4);

        while (leftDrive.isBusy()) {

        }

        leftDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive.setPower(0);
        rightDrive_fata.setPower(0);

        sleep(200);

    }

    public void line(int val, Consumer<Boolean> m) { //default forward

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - val);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - val);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition() - val);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition() - val);

        leftDrive.setPower(0.4);
        leftDrive_fata.setPower(0.4);
        rightDrive.setPower(0.4);
        rightDrive_fata.setPower(0.4);

        while (m.equals(Boolean.FALSE)) {

        }

        leftDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive.setPower(0);
        rightDrive_fata.setPower(0);

        sleep(200);

    }

    public static void strafe(int val) { //default left

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition()-val);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition()+val);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition()+val);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition()-val);

        leftDrive.setPower(0.7);
        leftDrive_fata.setPower(0.7);
        rightDrive.setPower(0.7);
        rightDrive_fata.setPower(0.7);

        while (leftDrive.isBusy()) {

        }

        leftDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive.setPower(0);
        rightDrive_fata.setPower(0);

        sleep(200);

    }

    public static void strafemore(double inch, DistanceSensor d) { //default left

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - inf);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + inf);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition() + inf);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition() - inf);

        leftDrive.setPower(0.7);
        leftDrive_fata.setPower(0.7);
        rightDrive.setPower(0.7);
        rightDrive_fata.setPower(0.7);

        while (d.getDistance(DistanceUnit.INCH) < inch) {

        }

        leftDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive.setPower(0);
        rightDrive_fata.setPower(0);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition());
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition());
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition());
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition());

        sleep(200);
    }

    public static void strafeless(double inch, DistanceSensor d) { //default left

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - inf);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + inf);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition() + inf);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition() - inf);

        leftDrive.setPower(0.7);
        leftDrive_fata.setPower(0.7);
        rightDrive.setPower(0.7);
        rightDrive_fata.setPower(0.7);

        while (d.getDistance(DistanceUnit.INCH) > inch) {

        }

        leftDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive.setPower(0);
        rightDrive_fata.setPower(0);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition());
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition());
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition());
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition());

        sleep(200);
    }
    public void raise_arm(double pts) {

    }
    public static class PrompDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the Promp position
         */
        public enum PrompPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        //Color variables
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(285,250);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(50,330);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(540,330);
        static final int REGION_WIDTH = 70;
        static final int REGION_HEIGHT = 100;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_HEIGHT,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_WIDTH);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        // Working variables
        Mat region1_hue, region2_hue, region3_hue;
        Mat Hsl = new Mat();
        Mat hue = new Mat();
        int avg1, avg2, avg3;

        void inputToHSL(Mat input)
        {
            Imgproc.cvtColor(input, Hsl, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(Hsl, hue, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToHSL(firstFrame);

            region1_hue = hue.submat(new Rect(region1_pointA, region1_pointB));
            region2_hue = hue.submat(new Rect(region2_pointA, region2_pointB));
            region3_hue = hue.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToHSL(input);



            //avg point value in the divided submat
            avg1 = (int) Core.mean(region1_hue).val[0];
            avg2 = (int) Core.mean(region2_hue).val[0];
            avg3 = (int) Core.mean(region3_hue).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    2);


            /*
             * Find the max of the 3 averages
             */
            int max = Math.max(avg1, Math.max(avg2, avg3));


            if(max == avg1) //centru
            {
                position = PrompDeterminationPipeline.PrompPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            }
            else if(max == avg2) // Was it from region 2? //stanga
            {
                position = PrompDeterminationPipeline.PrompPosition.CENTER;

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3)
            {
                position = PrompDeterminationPipeline.PrompPosition.RIGHT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public PrompDeterminationPipeline.PrompPosition getAnalysis()
        {
            return position;
        }
    }

    public static double mm2inch(int mm) {
        return mm / 25.4;
    }
    public static Pose2d getPosRedBoard() {
        double right, angle;
        right = dist_right.getDistance(DistanceUnit.INCH);


        angle = imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        return new Pose2d(72 - right - mm2inch(312), -(72 - 8.6), angle);
    }

    public static Pose2d getPosBlueBoard() {
        double left, angle;
        left = dist_right.getDistance(DistanceUnit.INCH);


        angle = imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        return new Pose2d(72 - left - mm2inch(312), 72 - 8.6, angle);
    }
    public static Pose2d getPosRedStack() {
        double left, angle;
        left = dist_right.getDistance(DistanceUnit.INCH);


        angle = imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        return new Pose2d(-(72 - left - mm2inch(312)), -(72 - 8.6), angle);
    }
    public static Pose2d getPosBlueStack() {
        double right, angle;
        right = dist_right.getDistance(DistanceUnit.INCH);

        angle = imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        return new Pose2d(-(72 - right - mm2inch(312)), 72 - 8.6, angle);
    }

    public static Boolean optimaldistleft(double inch) {
        return dist_left.getDistance(DistanceUnit.INCH) > inch;
    }


    public static Boolean optimaldistright(double inch) {
        return dist_left.getDistance(DistanceUnit.INCH) > inch;
    }

    public static int pixelnr() {
        return (pix1.alpha() < 0.01? 1 : 0) + (pix2.alpha() < 0.01? 1 : 0);
    }



}

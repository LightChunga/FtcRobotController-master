package org.firstinspires.ftc.teamcode.Beta;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

//@Autonomous
public class NeuProgram extends LinearOpMode {

    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor leftDrive_fata = null;
    DcMotor rightDrive_fata = null;
    DcMotor motorBrat = null;
    Servo gheara_s = null;
    Servo gheara_d = null;
    Servo spin = null;
    OpenCvCamera webc;
    NeuProgram.PrompDeterminationPipeline pipeline;
    IMU imm = null;

    final double PI = 3.14159265359;

    DistanceSensor dist = null;
    final double ppr223 = 751.8;
    final double tileln = 61;
    final double ppr435 = 384.5;
    public static volatile NeuProgram.PrompDeterminationPipeline.PrompPosition position = NeuProgram.PrompDeterminationPipeline.PrompPosition.LEFT;
    public void right_spin(int val) {

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive_fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive_fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(-val);
        rightDrive.setTargetPosition(val);
        leftDrive_fata.setTargetPosition(-val);
        rightDrive_fata.setTargetPosition(val);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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

    public void left_spin(int val) {

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive_fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive_fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(val);
        rightDrive.setTargetPosition(-val);
        leftDrive_fata.setTargetPosition(val);
        rightDrive_fata.setTargetPosition(-val);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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

    public void inainte(int val) {

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition()-val);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition()-val);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition()-val);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition()-val);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void stanga(int val) {

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition()-val);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + val);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition() + val);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition() - val);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void dreapta(int val) {

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + val);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - val);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition() - val);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition() + val);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void spate(int val) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + val);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + val);
        leftDrive_fata.setTargetPosition(leftDrive_fata.getCurrentPosition() + val);
        rightDrive_fata.setTargetPosition(rightDrive_fata.getCurrentPosition() + val);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive_fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
    public void raise_arm() {
        int aux = motorBrat.getCurrentPosition();

        while (abs(motorBrat.getCurrentPosition() - aux) <= ppr223) {
            motorBrat.setPower(-0.3);
        }

        motorBrat.setPower(0);
        sleep(500);
        gheara_s.setPosition(0.535);
        gheara_d.setPosition(0.535);

        while (motorBrat.getCurrentPosition() <= aux) {
            motorBrat.setPower(0.3);
        }

        motorBrat.setPower(0);
    }

    //int position = 0;

    @Override
    public void runOpMode() {
        leftDrive_fata = hardwareMap.dcMotor.get("Motor3");
        leftDrive = hardwareMap.dcMotor.get("Motor2");
        rightDrive_fata = hardwareMap.dcMotor.get("Motor4");
        rightDrive = hardwareMap.dcMotor.get("Motor1");
        motorBrat = hardwareMap.get(DcMotor.class, "Brat");
        dist = hardwareMap.get(DistanceSensor.class, "distsensor");
        imm = hardwareMap.get(IMU.class, "imu");

        //imm = hardwareMap.get();

        gheara_s = hardwareMap.get(Servo.class, "Srv");
        gheara_d = hardwareMap.get(Servo.class, "Srv2");

        spin = hardwareMap.get(Servo.class, "Srv1");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive_fata.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive_fata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive_fata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive_fata.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive_fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive_fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webc = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        pipeline = new NeuProgram.PrompDeterminationPipeline();
        webc.setPipeline(pipeline);

        webc.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webc.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        double initangle = imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while(!isStarted()) {
            telemetry.addData("Back Distance", dist.getDistance(DistanceUnit.CM));
            telemetry.addData("X:", imm.getRobotOrientationAsQuaternion().x);
            telemetry.addData("Y:", imm.getRobotOrientationAsQuaternion().y);
            telemetry.addData("Z:", imm.getRobotOrientationAsQuaternion().z);
            telemetry.addData("Yaw deg:", imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Yaw rad:", imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Init Yaw:", initangle);
            telemetry.update();
        }

        waitForStart();

        //initangle = imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double d = dist.getDistance(DistanceUnit.CM);

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.update();

        while (imm.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) != (initangle + PI/2) - PI) {
            leftDrive.setPower(0.4);
            leftDrive_fata.setPower(0.4);
            rightDrive.setPower(-0.4);
            rightDrive_fata.setPower(-0.4);
        }

        leftDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive.setPower(0);
        rightDrive_fata.setPower(0);

        int aux = 0;

        switch (position) {
            case LEFT: // center




                break;
            case RIGHT:



                break;
            case CENTER:

                break;
        }
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
                position = NeuProgram.PrompDeterminationPipeline.PrompPosition.LEFT; // Record our analysis

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
                position = NeuProgram.PrompDeterminationPipeline.PrompPosition.CENTER;

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3)
            {
                position = NeuProgram.PrompDeterminationPipeline.PrompPosition.RIGHT; // Record our analysis

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
        public NeuProgram.PrompDeterminationPipeline.PrompPosition getAnalysis()
        {
            return position;
        }
    }
}

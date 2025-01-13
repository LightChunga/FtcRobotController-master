package org.firstinspires.ftc.teamcode.Autonomous.New;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Autonomous.New.Util.Utils;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedRight extends LinearOpMode {

    ServoImplEx servospate = null;
    ServoImplEx servobk = null;
    DcMotorEx sliderleft = null;
    DcMotorEx sliderright = null;
    DcMotorEx actuator = null;
    ServoImplEx servobara = null;

    //ToDo: tune this later
    static final double pcm = 72;
    static final double rp = Math.sqrt(6) - Math.sqrt(2);
    static final double L0 = 29;

    void setarmheight(double cm, double pow) {
        double dif_h = rp * cm - L0;
        int pts = -(int)pcm * (int)dif_h;

        sliderleft.setTargetPosition(pts);
        sliderright.setTargetPosition(pts);

        sliderleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sliderleft.setPower(pow);
        sliderright.setPower(pow);
    }
    public void RaiseArm(double h0) {
        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(-0.9);
        sliderleft.setPower(-0.9);

        while(-sliderleft.getCurrentPosition() <= pts) {
            telemetry.addData("Current Encoder Position(right): ", -sliderright.getCurrentPosition());
            telemetry.addData("Current Encoder Position(left): ", -sliderleft.getCurrentPosition());
            telemetry.addData("Pts: ", pts);
            telemetry.update();
        }

        /*if (isStopRequested())
            exit(-1);*/

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    public void LowerArm(double h0) {
        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(0.9);
        sliderleft.setPower(0.9);

        while(-sliderleft.getCurrentPosition() >= pts) {
            telemetry.addData("Current Encoder Position(right): ", -sliderright.getCurrentPosition());
            telemetry.addData("Current Encoder Position(left): ", -sliderleft.getCurrentPosition());
            telemetry.addData("Pts: ", pts);
            telemetry.update();
        }

        /*if (isStopRequested())
            exit(-1);*/

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servospate = hardwareMap.get(ServoImplEx.class, "claw2");
        servobk = hardwareMap.get(ServoImplEx.class, "servobk");
        servobara = hardwareMap.get(ServoImplEx.class, "servo_bara");

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");

        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");

        actuator = hardwareMap.get(DcMotorEx.class, "Pivot");

        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servobara.setPosition(0);
        actuator.setPower(0.07);

        waitForStart();

        if (isStopRequested()) return;

        //Todo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(24.00, -60.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(0.00, -37.6))
                .build();

        TrajectorySequence towallcube = drive.trajectorySequenceBuilder(new Pose2d(43.75, -59.00, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(0.00, -39.00, Math.toRadians(270.00)), Math.toRadians(90.00))
                .build();

        TrajectorySequence closer = drive.trajectorySequenceBuilder(new Pose2d(0, -39, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(0, -38.1))
                .build();

        TrajectorySequence closerr = drive.trajectorySequenceBuilder(new Pose2d(0.00, -39.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(0.00, -33.00))
                .build();

        //todo: change starting vals
        TrajectorySequence firstcube1 = drive.trajectorySequenceBuilder(new Pose2d(0.00, -38.40, Math.toRadians(270.00)))
                .splineTo(new Vector2d(21.16, -56.55), Math.toRadians(0.00))
                .splineTo(new Vector2d(34.78, -34.78), Math.toRadians(90.00))
                .splineTo(new Vector2d(46.39, -8.51), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(49.43, -56.56))
                .splineToLinearHeading(new Pose2d(58.27, -40.72, Math.toRadians(90.00)), Math.toRadians(90.00))
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(44.28, -49.96), Math.toRadians(270.00))
                .lineToConstantHeading(new Vector2d(43.75, -59.30))
                .build();



        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(new Pose2d(0.00, -33.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(47.75, -55.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(42.82, -32.86), Math.toRadians(96.73))
                .splineTo(new Vector2d(57.27, -13.79), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(59.00, -56.20))
                .build();

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(59.00, -56.30, Math.toRadians(90.00)))
                .splineTo(new Vector2d(44.68, -21.84), Math.toRadians(53.13))
                .splineTo(new Vector2d(62.50, -11.29), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(62.23, -55.37))
                //.lineToConstantHeading(new Vector2d(50.49, -8.12))
                //.lineToConstantHeading(new Vector2d(24.09, -7.85))
                .build();

        drive.setPoseEstimate(towall.start());

        drive.followTrajectorySequence(towall);
        RaiseArm(80);
        servobara.setPosition(0.4);

        setarmheight(29, 0.8);

        drive.followTrajectorySequence(firstcube1);

        sliderright.setPower(0);
        sliderleft.setPower(0);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RaiseArm(31.4);
        sleep(1000);
        servobara.setPosition(0);
        sleep(1000);

        RaiseArm(3);
        drive.followTrajectorySequence(towallcube);
        RaiseArm(75);
        drive.followTrajectorySequence(closer);
        setarmheight(29, 0.9);
        drive.followTrajectorySequence(closerr);
        servobara.setPosition(0.4);

        drive.followTrajectorySequence(secondcube);
    }
}

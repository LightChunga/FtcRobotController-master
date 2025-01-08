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
public class BlueLeft extends LinearOpMode {

    ServoImplEx servospate = null;
    ServoImplEx servobk = null;
    ServoImplEx servobara = null;
    DcMotorEx sliderleft = null;
    DcMotorEx sliderright = null;
    DcMotorEx actuator = null;

    //ToDo: tune this later
    static final double pcm = 72;
    static final double rp = Math.sqrt(6) - Math.sqrt(2);
    static final double L0 = 29;
    final double pivcm = 8.722;

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

    void pivot(double cm, double pow) {
        int pts = -(int)(cm * pivcm);
        actuator.setTargetPosition(pts);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setPower(pow);
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

        waitForStart();

        if (isStopRequested()) return;

        //todo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(-24.00, 60.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(0., 38.1))
                .build();

        TrajectorySequence towallcube = drive.trajectorySequenceBuilder(new Pose2d(-43.75, 59.00, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(0.00, 39.00, Math.toRadians(90.00)), Math.toRadians(270.00))
                .build();

        TrajectorySequence closer = drive.trajectorySequenceBuilder(new Pose2d(0, 39, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(0, 38.1))
                .build();

        TrajectorySequence closerr = drive.trajectorySequenceBuilder(new Pose2d(0, 39, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(0, 33))
                .build();

        TrajectorySequence firstcube1 = drive.trajectorySequenceBuilder(new Pose2d(0.00, 38.40, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-21.16, 56.55), Math.toRadians(180.00))
                .splineTo(new Vector2d(-40.22, 40.08), Math.toRadians(294.07))
                .splineTo(new Vector2d(-47.88, 9.17), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-47.74, 55.10))
                .splineToLinearHeading(new Pose2d(-55.24, 46.66, Math.toRadians(-90.00)), Math.toRadians(-70.79))
                .splineToConstantHeading(new Vector2d(-44.28, 49.96), Math.toRadians(54.00))
                .lineToConstantHeading(new Vector2d(-43.75, 58.6))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(new Pose2d(0, 33, Math.toRadians(90)))
                .splineTo(new Vector2d(-47.75, 55.00), Math.toRadians(180.00))
                .splineTo(new Vector2d(-42.82, 32.86), Math.toRadians(263.27))
                .splineTo(new Vector2d(-57.27, 13.79), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-59.00, 56.20))
                .build();

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(-59.00, 56.30, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-42.04, 21.98), Math.toRadians(306.87))
                .splineTo(new Vector2d(-65.00, 11.42), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-64.00, 55.37))
                .lineToConstantHeading(new Vector2d(-46.00, 37.42))
                .splineToConstantHeading(new Vector2d(-24.22, 2.18), Math.toRadians(0.00))
                .build();

        drive.setPoseEstimate(towall.start());

        drive.followTrajectorySequence(towall);
        RaiseArm(80);
        servobara.setPosition(0.4);

        setarmheight(29, 0.8);
        /*while (sliderright.isBusy()) {
            telemetry.addData("Slider right: ", sliderright.getCurrentPosition());
            telemetry.addData("Slider left: ", sliderleft.getCurrentPosition());
            telemetry.update();
        }
        sleep(5000);*/

        drive.followTrajectorySequence(firstcube1);

        sliderright.setPower(0);
        sliderleft.setPower(0);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RaiseArm(30.5);
        servobara.setPosition(0);
        sleep(1000);

        RaiseArm(32);
        drive.followTrajectorySequence(towallcube);
        RaiseArm(75);
        drive.followTrajectorySequence(closer);
        setarmheight(29, 0.9);
        drive.followTrajectorySequence(closerr);
        servobara.setPosition(0.4);

        drive.followTrajectorySequence(secondcube);
        //drive.followTrajectorySequence(thirdcube);
    }
}

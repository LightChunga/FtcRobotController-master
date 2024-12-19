package org.firstinspires.ftc.teamcode.Autonomous.New;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.Autonomous.New.Util.Utils;


@Autonomous
public class AutoRedLeft extends LinearOpMode {
    ServoImplEx servospate = null;
    ServoImplEx servobk = null;
    DcMotorEx sliderleft = null;
    DcMotorEx sliderright = null;

    //ToDo: tune this later
    final double pcm = 72;
    final double rp = Math.sqrt(6) - Math.sqrt(2);
    final double L0 = 29;

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

    public void RaiseArm(double h0, Telemetry telemetry) {
        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(-0.9);
        sliderleft.setPower(-0.9);

        while(-sliderright.getCurrentPosition() <= pts) {
            telemetry.addData("Current Encoder Position: ", -sliderright.getCurrentPosition());
            telemetry.addData("Pts: ", pts);
            telemetry.update();
        }

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    public void LowerArm(double h0, Telemetry telemetry) {
        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(0.9);
        sliderleft.setPower(0.9);

        while(-sliderright.getCurrentPosition() >= pts) {
            telemetry.addData("Current Encoder Position: ", -sliderright.getCurrentPosition());
            telemetry.addData("Pts: ", pts);
            telemetry.update();
        }

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servospate = hardwareMap.get(ServoImplEx.class, "claw2");
        servobk = hardwareMap.get(ServoImplEx.class, "servobk");

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");

        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");

        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servospate.setPosition(0.45);
        servobk.setPosition(0.47);

        //ToDo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(-24, -60.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(-8, -41))
                .build();

        TrajectorySequence tocubes = drive.trajectorySequenceBuilder(new Pose2d(-8.00, -41.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-56.11, -43.98), Math.toRadians(70.00))
                .build();

        TrajectorySequence firstcube = drive.trajectorySequenceBuilder(new Pose2d(-6.00, -40, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-37.77, -40.80), Math.toRadians(90.00))
                .splineTo(new Vector2d(-47.45, -9.75), Math.toRadians(180.00))
                .lineTo(new Vector2d(-48, -60))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(firstcube.end())
                .splineToConstantHeading(new Vector2d(-47.88, -9.60), Math.toRadians(180.00))
                .splineTo(new Vector2d(-56.55, -9.32), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-58, -60))
                .build();

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(-58.00, -60.00, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(-50.91, -29.39))
                .lineToConstantHeading(new Vector2d(-61.46, -9.89))
                .lineToConstantHeading(new Vector2d(-61.75, -59.29))
                .lineToConstantHeading(new Vector2d(-41.52, -40.08))
                .splineToConstantHeading(new Vector2d(-27.51, -11.77), Math.toRadians(0.00))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(towall.start());

        drive.followTrajectorySequence(towall);
        RaiseArm(80, telemetry);
        /*while (sliderright.isBusy() && sliderleft.isBusy()) {}
        sleep(100);
        servospate.setPosition(0);
        setarmheight(29, -1.0);
        drive.followTrajectorySequence(tocubes);
        while (sliderright.isBusy() && sliderleft.isBusy()) {}*/
    }
}

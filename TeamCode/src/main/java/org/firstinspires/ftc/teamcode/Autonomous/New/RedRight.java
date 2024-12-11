package org.firstinspires.ftc.teamcode.Autonomous.New;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.Autonomous.New.Util.Utils;


@Autonomous
public class RedRight extends LinearOpMode {

    ServoImplEx servospate = null;
    DcMotorEx sliderleft = null;
    DcMotorEx sliderright = null;
    DcMotorEx encoder_arm = null;

    //ToDo: tune this later
    final double pcm = 72;
    final double rp = Math.sqrt(6) - Math.sqrt(2);
    final double L0 = 29;
    public void RaiseArm(double h0) {
        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(-0.7);
        sliderleft.setPower(-0.7);

        while(-encoder_arm.getCurrentPosition() <= pts) {
            telemetry.addData("Current Encoder Position: ", -encoder_arm.getCurrentPosition());
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

        sliderright.setPower(0.7);
        sliderleft.setPower(0.7);

        while(-encoder_arm.getCurrentPosition() >= pts) {
            telemetry.addData("Current Encoder Position: ", -encoder_arm.getCurrentPosition());
            telemetry.addData("Pts: ", pts);
            telemetry.update();
        }

        /*if (isStopRequested())
            exit(-1);*/

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servospate = hardwareMap.get(ServoImplEx.class, "servospate");

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");

        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");

        encoder_arm = hardwareMap.get(DcMotorEx.class, "armencoder");

        encoder_arm.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servospate.setPosition(0.1);

        telemetry.addData("Init encoder position: ", -encoder_arm.getCurrentPosition());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        //ToDo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(-24, -60.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(-8.0, -38.40))
                .build();

        TrajectorySequence firstcube = drive.trajectorySequenceBuilder(new Pose2d(-8.00, -38.40, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-37.77, -40.80), Math.toRadians(90.00))
                .splineTo(new Vector2d(-47.45, -9.75), Math.toRadians(180.00))
                .lineTo(new Vector2d(-48, -60))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(firstcube.end())
                .splineToConstantHeading(new Vector2d(-47.88, -9.60), Math.toRadians(180.00))
                .splineTo(new Vector2d(-56.55, -9.32), Math.toRadians(179.24))
                .lineToConstantHeading(new Vector2d(-58, -60))
                .build();

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(-58.00, -60.00, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(-50.91, -29.39))
                .lineToConstantHeading(new Vector2d(-61.46, -9.89))
                .lineToConstantHeading(new Vector2d(-61.75, -59.29))
                .lineToConstantHeading(new Vector2d(-41.52, -40.08))
                .splineToConstantHeading(new Vector2d(-27.51, -11.77), Math.toRadians(0.00))
                .build();


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(towall.end())
                .lineToConstantHeading(new Vector2d(-38.36, -8.87))
                .lineToConstantHeading(new Vector2d(-49.30, -10.69))
                .lineToConstantHeading(new Vector2d(-48.80, -49.30))
                .lineToConstantHeading(new Vector2d(-54.60, -56.59))
                .lineToConstantHeading(new Vector2d(-43.17, -36.70))
                .lineToConstantHeading(new Vector2d(-44.00, -13.01))
                .lineToConstantHeading(new Vector2d(-56.48, -13.12))
                .lineToConstantHeading(new Vector2d(-59.24, -54.43))
                .lineToConstantHeading(new Vector2d(-58.88, -9.23))
                .lineTo(new Vector2d(-62.77, -9.37))
                .lineTo(new Vector2d(-62.50, -54.47))
                .lineTo(new Vector2d(-47.24, -33.32))
                .splineTo(new Vector2d(-26.63, -12.04), Math.toRadians(0.00))
                .build();


        drive.setPoseEstimate(towall.start());

        drive.followTrajectorySequence(towall);

        RaiseArm(80);
        //sleep(8000);
        servospate.setPosition(0);
        sleep(200);
        LowerArm(29);

        drive.followTrajectorySequence(firstcube);
        telemetry.addData("firstcube", 1);
        telemetry.update();
        drive.followTrajectorySequence(secondcube);
        telemetry.addData("secondcube", 1);
        telemetry.update();
        drive.followTrajectorySequence(thirdcube);
        telemetry.addData("thirdcube", 1);
        telemetry.update();
    }
}

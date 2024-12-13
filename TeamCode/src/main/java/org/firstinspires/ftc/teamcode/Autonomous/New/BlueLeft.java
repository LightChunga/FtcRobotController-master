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
    DcMotorEx sliderleft = null;
    DcMotorEx sliderright = null;
    DcMotorEx encoder_arm = null;
    DcMotorEx actuator = null;

    //ToDo: tune this later
    static final double pcm = 72;
    static final double rp = Math.sqrt(6) - Math.sqrt(2);
    static final double L0 = 29;

    public void RaiseArm(double h0) {
        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(-0.9);
        sliderleft.setPower(-0.9);

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

        sliderright.setPower(0.9);
        sliderleft.setPower(0.9);

        while(-encoder_arm.getCurrentPosition() >= pts) {
        }

        /*if (isStopRequested())
            exit(-1);*/

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servospate = hardwareMap.get(ServoImplEx.class, "servospate");

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");

        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");

        encoder_arm = hardwareMap.get(DcMotorEx.class, "armencoder");

        actuator = hardwareMap.get(DcMotorEx.class, "Pivot");

        encoder_arm.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servospate.setPosition(0.12);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(-24.00, 60.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(0, 38.4))
                .build();

        TrajectorySequence towallcube = drive.trajectorySequenceBuilder(new Pose2d(0, 42.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(0, 38.40))
                .build();

        TrajectorySequence firstcube = drive.trajectorySequenceBuilder(new Pose2d(-0.00, 38.40, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-27.37, 38.78), Math.toRadians(180.93))
                .splineTo(new Vector2d(-46.87, 5.71), Math.toRadians(180.00))
                .lineTo(new Vector2d(-47.16, 56.69))
                .splineToLinearHeading(new Pose2d(-54.69, 47.65, Math.toRadians(270.00)), Math.toRadians(227.11))
                .splineToConstantHeading(new Vector2d(-60.0, 57), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(-24.00, 57.00))
                .build();

        //applied twice
        TrajectorySequence towall2 = drive.trajectorySequenceBuilder(new Pose2d(-24.00, 57.00, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(0, 42.00, Math.toRadians(90.00)))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(new Pose2d(-0.00, 38.40, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-27.37, 38.78), Math.toRadians(180.93))
                .splineTo(new Vector2d(-57.64, 12.74), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-60.00, 57.00))
                .turn(90)
                .lineToConstantHeading(new Vector2d(-24.00, 57.00))
                .build();

        //towall2

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(-0.00, 38.40, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-27.37, 38.78), Math.toRadians(180.93))
                .splineTo(new Vector2d(-60.88, 11.89), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-60.00, 58.00))
                .turn(90)
                .lineToConstantHeading(new Vector2d(-24.00, 57.00))
                .build();

        //towall2

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(0, 38.40, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-42.10, 37.62), Math.toRadians(222.39))
                .splineToLinearHeading(new Pose2d(-25.78, 10.18, Math.toRadians(270.00)), Math.toRadians(0.00))
                .build();

        drive.setPoseEstimate(towall.start());
        drive.followTrajectorySequence(towall);
        RaiseArm(80);
        servospate.setPosition(0);
        LowerArm(29);

        drive.followTrajectorySequence(firstcube);
        servospate.setPosition(0.12);
        drive.followTrajectorySequence(towall2);
        RaiseArm(70);
        drive.followTrajectorySequence(towallcube);
        RaiseArm(80);
        servospate.setPosition(0);
        LowerArm(29);

        /*drive.followTrajectorySequence(secondcube);
        servospate.setPosition(0.12);
        drive.followTrajectorySequence(towall2);
        RaiseArm(70);
        drive.followTrajectorySequence(towallcube);
        RaiseArm(80);
        servospate.setPosition(0);
        LowerArm(29);

        drive.followTrajectorySequence(thirdcube);
        servospate.setPosition(0.12);
        drive.followTrajectorySequence(towall2);
        RaiseArm(70);
        drive.followTrajectorySequence(towallcube);
        RaiseArm(80);
        servospate.setPosition(0);
        LowerArm(29);*/

        drive.followTrajectorySequence(park);
    }
}

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

        servospate.setPosition(0.23);

        waitForStart();

        if (isStopRequested()) return;

        //todo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(-24.00, 60.00, Math.toRadians(90.00)))
                .waitSeconds(4)
                .lineToConstantHeading(new Vector2d(-8, 37.7))
                .build();

        TrajectorySequence towallcube = drive.trajectorySequenceBuilder(new Pose2d(0, 42.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(0, 38.40))
                .build();

        TrajectorySequence firstcube1 = drive.trajectorySequenceBuilder(new Pose2d(-8, 37.7, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-21.16, 56.55), Math.toRadians(180.00))
                .splineTo(new Vector2d(-40.22, 40.08), Math.toRadians(294.07))
                .splineTo(new Vector2d(-47.88, 9.17), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-47.74, 55.10))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(new Pose2d(-47.74, 55.10, Math.toRadians(180.00)))
                .splineTo(new Vector2d(-42.82, 32.86), Math.toRadians(263.27))
                .splineTo(new Vector2d(-57.27, 13.79), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-59.00, 56.30))
                .build();


        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(-59.00, 56.30, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-62.61, 14.23), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-62.32, 56.26))
                .lineToConstantHeading(new Vector2d(-31.70, 10.90))
                .build();


        drive.setPoseEstimate(towall.start());

        drive.followTrajectorySequence(towall);
        RaiseArm(80);
        servospate.setPosition(0);
        LowerArm(29);



        //drive.followTrajectorySequence(firstcube);
    }
}

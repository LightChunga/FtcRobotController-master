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
    ServoImplEx bclaw = null;
    ServoImplEx claw = null;
    ServoImplEx claw2 = null;
    ServoImplEx servobk = null;
    ServoImplEx servospate = null;
    ServoImplEx servobara = null;
    DcMotorEx sliderleft = null;
    DcMotorEx sliderright = null;
    DcMotorEx actuator = null;

    //ToDo: tune this later
    final double pcm = 71;
    final double rp = Math.sqrt(6) - Math.sqrt(2);
    final double L0 = 29;
    final double pivcm = 7.7611;

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

    public void RaiseArm(double h0, Telemetry telemetry) {

        sliderleft.setPower(0);
        sliderright.setPower(0);

        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(-0.9);
        sliderleft.setPower(-0.9);

        while(-sliderright.getCurrentPosition() <= pts) {
            telemetry.addData("Current Encoder Pos(right): ", -sliderright.getCurrentPosition());
            telemetry.addData("Current Encoder Pos(left): ", -sliderleft.getCurrentPosition());
            telemetry.addData("Pts: ", pts);
            telemetry.update();
        }

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    public void LowerArm(double h0, Telemetry telemetry) {
        sliderleft.setPower(0);
        sliderright.setPower(0);

        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(0.9);
        sliderleft.setPower(0.9);

        while(-sliderright.getCurrentPosition() >= pts) {
            telemetry.addData("Current Encoder Pos(right): ", -sliderright.getCurrentPosition());
            telemetry.addData("Current Encoder Pos(left): ", -sliderleft.getCurrentPosition());
            telemetry.addData("Pts: ", pts);
            telemetry.update();
        }

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        claw = hardwareMap.get(ServoImplEx.class, "claw"); //hook
        bclaw = hardwareMap.get(ServoImplEx.class, "bclaw");
        claw2 = hardwareMap.get(ServoImplEx.class, "claw2");
        servobk = hardwareMap.get(ServoImplEx.class, "servobk");
        servospate = hardwareMap.get(ServoImplEx.class, "servospate");
        servobara = hardwareMap.get(ServoImplEx.class, "servo_bara");

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");
        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");
        actuator = hardwareMap.get(DcMotorEx.class, "Pivot");


        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw2.setPosition(0.45);//gheara spate inchis
        servobk.setPosition(0.1);//poz gheara spate transfer

        //ToDo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(-24, -60.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(-8, -37.9))
                .build();

        TrajectorySequence firstcube = drive.trajectorySequenceBuilder(new Pose2d(-24.00, -60.00, Math.toRadians(0.00)))
                .splineToLinearHeading(new Pose2d(-45.73, -45.34, Math.toRadians(45.00)), Math.toRadians(225.00))
                .build();

        TrajectorySequence closer = drive.trajectorySequenceBuilder(new Pose2d(-45.73, -45.34, Math.toRadians(45.00)))
                .lineToConstantHeading(new Vector2d(-51.15, -50.62))
                .build();

        TrajectorySequence further = drive.trajectorySequenceBuilder(new Pose2d(-51.15, -50.62, Math.toRadians(45.00)))
                .splineTo(new Vector2d(-48.11, -46.66), Math.toRadians(48.79))
                .build();

        TrajectorySequence fcube = drive.trajectorySequenceBuilder(new Pose2d(-48.11, -46.66, Math.toRadians(48.79)))
                .splineTo(new Vector2d(-49.30, -40.45), Math.toRadians(90.00))
                .build();

        TrajectorySequence firstintake = drive.trajectorySequenceBuilder(new Pose2d(-8.00, -38.10, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-51, -40.85), Math.toRadians(90.00))
                .build();

        TrajectorySequence tocubes = drive.trajectorySequenceBuilder(new Pose2d(-8.00, -37.70, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-51, -46.26), Math.toRadians(90.00))
                .build();

        TrajectorySequence nfirstcube = drive.trajectorySequenceBuilder(new Pose2d(-48.31, -43.98, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-58.28, -43.84))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -60.00, Math.toRadians(180.00)))
                .splineToConstantHeading(new Vector2d(-40.98, -35.18), Math.toRadians(90.38))
                .splineToConstantHeading(new Vector2d(-47.88, -9.60), Math.toRadians(180.00))
                .splineTo(new Vector2d(-56.55, -9.32), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-58.00, -60.00))
                .build();

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(-58.00, -60.00, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(-50.91, -29.39))
                .lineToConstantHeading(new Vector2d(-63.16, -9.83))
                .lineToConstantHeading(new Vector2d(-62.89, -59.07))
                .lineToConstantHeading(new Vector2d(-51.94, -11.29))
                .lineToConstantHeading(new Vector2d(-28.18, -10.10))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(firstcube.start());

        setarmheight(109, 0.999);
        drive.followTrajectorySequence(firstcube);
        servobk.setPosition(0.6);
        while (sliderleft.isBusy() && sliderright.isBusy()) {}
        drive.followTrajectorySequence(closer);
        claw2.setPosition(0);
        servobk.setPosition(0.8);
        drive.followTrajectorySequence(further);
        setarmheight(29, -0.999);
        sleep(2000);
        drive.followTrajectorySequence(fcube);
        while (sliderleft.isBusy() && sliderright.isBusy()) {}
        sliderleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivot(8, 0.999);
        while (actuator.isBusy()) {}
        bclaw.setPosition(0.02);//preluare
        servospate.setPosition(0.18);//poz preluare
        sleep(500);
        claw.setPosition(0.35);
        sleep(500);
        bclaw.setPosition(0.7);//transfer
        claw2.setPosition(0.0);//gheara spate deschis
        servospate.setPosition(0.02); //pozitie transfer

        servobk.setPosition(0.1);//poz gheara spate transfer

        pivot(0.1, -0.999);
        while (actuator.isBusy()) {}

        claw2.setPosition(0.45);
    }
}

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
    ServoImplEx clawrot = null;
    ServoImplEx claw_arm = null;
    ServoImplEx claw = null;
    ServoImplEx servobk = null;
    DcMotorEx sliderleft = null;
    DcMotorEx sliderright = null;
    DcMotorEx actuator = null;

    //ToDo: tune this later
    final double pcm = 72;
    final double rp = Math.sqrt(6) - Math.sqrt(2);
    final double L0 = 29;
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

        servospate = hardwareMap.get(ServoImplEx.class, "claw2");
        servobk = hardwareMap.get(ServoImplEx.class, "servobk");

        clawrot = hardwareMap.get(ServoImplEx.class, "servospate");
        claw_arm = hardwareMap.get(ServoImplEx.class, "bclaw");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");
        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");
        actuator = hardwareMap.get(DcMotorEx.class, "Pivot");

        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servospate.setPosition(0.45); //where the cube is
        servobk.setPosition(0.53); //tilt the claw at a 75 degree angle

        //ToDo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(-24, -60.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(-8, -37.9))
                .build();

        TrajectorySequence tocubes = drive.trajectorySequenceBuilder(new Pose2d(-8.00, -37.70, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-51, -46.26), Math.toRadians(90.00))
                .build();

        TrajectorySequence nfirstcube = drive.trajectorySequenceBuilder(new Pose2d(-48.31, -43.98, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-58.28, -43.84))
                .build();

        TrajectorySequence firstcube = drive.trajectorySequenceBuilder(new Pose2d(-8.00, -37.7, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-37.77, -40.80), Math.toRadians(90.00))
                .splineTo(new Vector2d(-47.45, -9.75), Math.toRadians(180.00))
                .lineTo(new Vector2d(-48, -60))
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

        drive.setPoseEstimate(towall.start());

        drive.followTrajectorySequence(towall);

        RaiseArm(78, telemetry);
        servospate.setPosition(0);
        setarmheight(29, 0.4);

        drive.followTrajectorySequence(firstcube);
        drive.followTrajectorySequence(secondcube);
        drive.followTrajectorySequence(thirdcube);

        /*RaiseArm(78, telemetry);
        sleep(500);
        servospate.setPosition(0);
        servobk.setPosition(0);
        setarmheight(29, -0.4);
        drive.followTrajectorySequence(tocubes);

        pivot(17, 0.5);

        while (actuator.isBusy()) {
            telemetry.addData("Current Encoder Pos(right): ", -sliderright.getCurrentPosition());
            telemetry.addData("Current Encoder Pos(left): ", -sliderleft.getCurrentPosition());
            telemetry.addData("pivot pos: ", actuator.getCurrentPosition());
            telemetry.update();
        }

        //todo: future project
        /*servobk.setPosition(0.1);
        servospate.setPosition(0);

        claw.setPosition(0);
        claw_arm.setPosition(0.02); //0.7; bclaw
        sleep(1000);
        claw.setPosition(0.35);// 0; claw
        sleep(1000);

        //servo spate --- claw2
        claw_arm.setPosition(0.7); //0.7
        clawrot.setPosition(0.1);
        sleep(1000);
        claw.setPosition(0);
        sleep(1000);
        servospate.setPosition(0.45);
        clawrot.setPosition(0.22);
        servobk.setPosition(0.7);
        pivot(0.5, -0.5);

        RaiseArm(105, telemetry);
        servobk.setPosition(0.6);
        sleep(500);

        drive.turn(Math.toRadians(-50));
        servospate.setPosition(0);

        drive.turn(Math.toRadians(50));
        setarmheight(29, -0.4);

        //servospate.setPosition();*/
    }
}

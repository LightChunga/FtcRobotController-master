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
public class BlueRight extends LinearOpMode {
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

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");

        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");

        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servospate.setPosition(0.45); //where the cube is
        servobk.setPosition(0.53); //tilt the claw at a 75 degree angle

        //ToDo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(24.00, 60.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(8.00, 38.4))
                .build();

        TrajectorySequence tocubes = drive.trajectorySequenceBuilder(new Pose2d(8.00, 38.4, Math.toRadians(90.00)))
                .splineTo(new Vector2d(51.78, 41.24), Math.toRadians(250.00))
                .build();

        TrajectorySequence firstcube = drive.trajectorySequenceBuilder(new Pose2d(-6.00, 37.90, Math.toRadians(90.00)))
                .splineTo(new Vector2d(37.77, 40.80), Math.toRadians(270.00))
                .splineTo(new Vector2d(47.45, 9.75), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(48.00, 60.00))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(new Pose2d(48.00, 60.00, Math.toRadians(0.00)))
                .splineTo(new Vector2d(43.36, 37.68), Math.toRadians(238.56))
                .splineTo(new Vector2d(44.94, 15.64), Math.toRadians(0.00))
                .splineTo(new Vector2d(56.55, 9.32), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(58.00, 54.00))
                .build();

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(58.00, 54.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(50.91, 29.39))
                .lineToConstantHeading(new Vector2d(62.10, 9.83))
                .lineToConstantHeading(new Vector2d(61.75, 54.00))
                .lineToConstantHeading(new Vector2d(58.27, 10.63))
                .lineToConstantHeading(new Vector2d(28.71, 9.97))
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

        RaiseArm(45, telemetry);
    }
}

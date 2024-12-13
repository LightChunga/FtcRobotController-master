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


@Autonomous
public class BlueRight extends LinearOpMode {

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
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(24.00, 60.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(8.00, 38.40))
                .build();

        TrajectorySequence firstcube = drive.trajectorySequenceBuilder(new Pose2d(8.00, 38.40, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(21.59, 55.25), Math.toRadians(-50.00))
                .splineToConstantHeading(new Vector2d(32.28, 39.07), Math.toRadians(-50.11))
                .splineToConstantHeading(new Vector2d(48.75, 13.36), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(49.00, 58.00))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(new Pose2d(49.00, 58.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(41.96, 38.64))
                .splineToConstantHeading(new Vector2d(38.78, 8.16), Math.toRadians(-61.00))
                .splineToConstantHeading(new Vector2d(56.55, 3.83), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(57.00, 57.00))
                .build();

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(57.00, 59.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(36.76, 36.76, Math.toRadians(270.00)))
                .splineTo(new Vector2d(61.31, 11.92), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(62.03, 55.68))
                .lineToConstantHeading(new Vector2d(42.39, 28.09))
                .splineToConstantHeading(new Vector2d(28.67, 9.60), Math.toRadians(180.00))
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

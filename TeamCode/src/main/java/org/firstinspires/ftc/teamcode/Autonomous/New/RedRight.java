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

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");

        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");

        actuator = hardwareMap.get(DcMotorEx.class, "Pivot");

        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servospate.setPosition(0.45); //where the cube is
        servobk.setPosition(0.53);

        waitForStart();

        if (isStopRequested()) return;

        //Todo
        TrajectorySequence towall = drive.trajectorySequenceBuilder(new Pose2d(24.00, -60.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(8, -37.7))
                .build();

        //todo: change starting vals
        TrajectorySequence firstcube1 = drive.trajectorySequenceBuilder(new Pose2d(8.00, -37.70, Math.toRadians(270.00)))
                .splineTo(new Vector2d(25.80, -58.54), Math.toRadians(0.00))
                .splineTo(new Vector2d(37.95, -21.71), Math.toRadians(65.93))
                .splineTo(new Vector2d(46.26, -9.57), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(48.00, -56.30))
                .build();

        TrajectorySequence secondcube = drive.trajectorySequenceBuilder(new Pose2d(48.00, -56.30, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(45.07, -9.83), Math.toRadians(0.00))
                .splineTo(new Vector2d(57.48, -9.44), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(59.00, -56.30))
                .build();

        TrajectorySequence thirdcube = drive.trajectorySequenceBuilder(new Pose2d(59.00, -56.30, Math.toRadians(90.00)))
                .splineTo(new Vector2d(44.68, -21.84), Math.toRadians(53.13))
                .splineTo(new Vector2d(62.50, -11.29), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(62.23, -55.37))
                .lineToConstantHeading(new Vector2d(49.56, -29.90))
                .splineToConstantHeading(new Vector2d(32.40, -9.70), Math.toRadians(136.47))
                .build();

        drive.setPoseEstimate(towall.start());
        drive.followTrajectorySequence(towall);
        RaiseArm(80);
        servospate.setPosition(0);
        LowerArm(29);

        drive.followTrajectorySequence(firstcube1);
        drive.followTrajectorySequence(secondcube);
        drive.followTrajectorySequence(thirdcube);

        /*
        TrajectorySequence firstcube = drive.trajectorySequenceBuilder(new Pose2d(0.00, -38.40, Math.toRadians(270.00)))
.splineTo(new Vector2d(25.80, -58.54), Math.toRadians(0.00))
.splineTo(new Vector2d(32.67, -37.15), Math.toRadians(65.93))
.splineTo(new Vector2d(48.77, -9.70), Math.toRadians(0.00))
.lineToConstantHeading(new Vector2d(48.90, -48.51))
.splineToLinearHeading(new Pose2d(60.38, -57.75, Math.toRadians(90.00)), Math.toRadians(270.00))
.build();

         */
    }
}

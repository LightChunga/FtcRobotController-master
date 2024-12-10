package org.firstinspires.ftc.teamcode.Autonomous.New.Util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Utils {
    public static ServoImplEx servospate = null;
    public static DcMotorEx sliderleft = null;
    public static DcMotorEx sliderright = null;
    public static DcMotorEx encoder_arm = null;

    //ToDo: tune this later
    static final double pcm = 68;
    static final double rp = Math.sqrt(6) - Math.sqrt(2);
    static final double L0 = 29;
    static public void RaiseArm(double h0) {
        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(-0.7);
        sliderleft.setPower(-0.7);

        while(-encoder_arm.getCurrentPosition() <= pts) {
        }

        /*if (isStopRequested())
            exit(-1);*/

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }

    static public void LowerArm(double h0) {
        double dif_h = rp * h0 - L0;

        int pts = (int)pcm * (int)dif_h;

        //double pwr = 0;

        sliderright.setPower(0.7);
        sliderleft.setPower(0.7);

        while(-encoder_arm.getCurrentPosition() >= pts) {
        }

        /*if (isStopRequested())
            exit(-1);*/

        sliderright.setPower(0);
        sliderleft.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode.TeleOpFiles;

import static java.lang.Double.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//ToDo: tune limits after motor replacement
@TeleOp
public class Test extends LinearOpMode {
    DcMotorEx actuator = null;

    double pivcm = 8.722;
    void pivot(double cm, double pow) {
        int pts = (int)(cm * pivcm);
        actuator.setTargetPosition(pts);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setPower(pow);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        actuator = hardwareMap.get(DcMotorEx.class, "Pivot");

        boolean actup, actdown;

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            actup = gamepad2.dpad_up;
            actdown = gamepad2.dpad_down;

            if (actup && !actdown) {
                actuator.setTargetPosition(-50);
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                actuator.setPower(0.5);
            }
            else if (!actup && actdown) {
                actuator.setTargetPosition(-100);
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                actuator.setPower(0.5);
            }

            if(isStopRequested()) return;
        }
    }
}

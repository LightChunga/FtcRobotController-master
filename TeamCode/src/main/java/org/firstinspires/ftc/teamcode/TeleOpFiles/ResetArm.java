package org.firstinspires.ftc.teamcode.TeleOpFiles;

import static java.lang.Double.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//ToDo: tune limits after motor replacement
@TeleOp
public class ResetArm extends LinearOpMode {
    Servo clawL = null;
    Servo clawR = null;
    ServoImplEx claw = null;
    DcMotor actuator = null;

    //theoretical values
    static final double b90 = 0;
    static final double dunit = 87.5;
    static final double b10 = -7000;
    void setarm(int ang) {

        double stop = max(b10, b90 - ang * dunit);

        if (actuator.getCurrentPosition() > stop) {
            actuator.setPower(-0.61);
            while (actuator.getCurrentPosition() > stop) {
                telemetry.addData("Current pivot poz:", actuator.getCurrentPosition());
                telemetry.update();
            }
        }
        else {
            actuator.setPower(0.81);
            while (actuator.getCurrentPosition() < stop) {
                telemetry.addData("Current pivot poz:", actuator.getCurrentPosition());
                telemetry.update();
            }
        }
        actuator.setPower(0);
    }

    void setarm_aux(double stop) {

        //double stop = -2250;

        if (actuator.getCurrentPosition() > stop) {
            actuator.setPower(-0.61);
            while (actuator.getCurrentPosition() > stop) {
                telemetry.addData("Current pivot poz:", actuator.getCurrentPosition());
                telemetry.update();
            }
        }
        else {
            actuator.setPower(0.81);
            while (actuator.getCurrentPosition() < stop) {
                telemetry.addData("Current pivot poz:", actuator.getCurrentPosition());
                telemetry.update();
            }
        }
        actuator.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        actuator = hardwareMap.get(DcMotor.class, "Pivot");
        claw = hardwareMap.get(ServoImplEx.class, "clawcon"); //hook

        actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        actuator.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry.addData("Make sure the arm is at an angle of 90 degrees then press start\nCurrent encoder pos: ", actuator.getCurrentPosition());
        telemetry.update();

        boolean actup, actdown;

        while (!opModeIsActive()) {
            actup = gamepad2.dpad_up;
            actdown = gamepad2.dpad_down;

            if (actup && !actdown)
                actuator.setPower(0.81);
            else if (!actup && actdown)
                actuator.setPower(-0.61);
            else
                actuator.setPower(0);

            telemetry.addData("Make sure the arm is at an angle of 90 degrees then press start\nCurrent encoder pos: ", actuator.getCurrentPosition());
            telemetry.update();

            if(isStopRequested()) return;
        }

        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setarm(30);
    }
}

package org.firstinspires.ftc.teamcode.TeleOpFiles;

import static java.lang.Double.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class teleopk extends LinearOpMode {

    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor leftDrive_fata = null;
    DcMotor rightDrive_fata = null;
    DcMotorEx sliderleft = null;
    DcMotorEx sliderright = null;
    DcMotorEx actuator = null;

    DcMotorEx encoder_arm = null;
    ServoImplEx bclaw = null;
    ServoImplEx claw = null;
    ServoImplEx claw2 = null;
    ServoImplEx servobk = null;
    ServoImplEx servospate = null;
    ServoImplEx servobara = null;
    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive_fata = hardwareMap.dcMotor.get("MotorLeftF");
        leftDrive = hardwareMap.dcMotor.get("MotorLeftB");
        rightDrive_fata = hardwareMap.dcMotor.get("MotorRightF");
        rightDrive = hardwareMap.dcMotor.get("MotorRightB");

        sliderleft = hardwareMap.get(DcMotorEx.class, "SliderLeft");
        sliderright = hardwareMap.get(DcMotorEx.class, "SliderRight");
        actuator = hardwareMap.get(DcMotorEx.class, "Pivot");

        claw = hardwareMap.get(ServoImplEx.class, "claw"); //hook
        bclaw = hardwareMap.get(ServoImplEx.class, "bclaw");
        claw2 = hardwareMap.get(ServoImplEx.class, "claw2");
        servobk = hardwareMap.get(ServoImplEx.class, "servobk");
        servospate = hardwareMap.get(ServoImplEx.class, "servospate");
        servobara = hardwareMap.get(ServoImplEx.class, "servo_bara");

        rightDrive_fata.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderleft.setDirection(DcMotorSimple.Direction.REVERSE);
        actuator.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive_fata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive_fata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //claw.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive_fata.setPower(0);
        rightDrive_fata.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive_fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive_fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");

        double x, y, z, up, down;
        boolean actup, actdown;
        double deadzone = 0.2;
        boolean a1,b1,a, b, upservo, downservo, su,sd,sb1,sb2;

        telemetry.addData("Current pivot poz:", actuator.getCurrentPosition());

        telemetry.update();
        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.1;

            z = gamepad1.right_stick_x;

            a1 = gamepad2.a;
            b1 = gamepad2.b;

            a = gamepad1.a; // servo close/open
            b = gamepad1.b;

            up = gamepad2.right_trigger;
            down = -gamepad2.left_trigger;

            sb1= gamepad2.y;
            sb2 = gamepad2.x;

            actup = gamepad1.dpad_up;
            actdown = gamepad1.dpad_down;

            downservo = gamepad1.left_bumper;
            upservo = gamepad1.right_bumper;

            su = gamepad2.left_bumper;
            sd = gamepad2.right_bumper;
            if(Math.abs(x)<deadzone) x = 0;
            if(Math.abs(y)<deadzone) y = 0;
            if(Math.abs(z)<deadzone) z = 0;

            //ToDo
            rightDrive_fata.setPower(Range.clip(y - x + z, -1, 1));
            leftDrive.setPower(Range.clip(y - x - z, -1, 1));
            leftDrive_fata.setPower(Range.clip(y + x - z, -1, 1)); //
            rightDrive.setPower(Range.clip(y + x+z, -1, 1));

            /*if(sb1 && !sb2){
                servospate.setPosition(0.1); //poz preluare
            } else if(!sb1 && sb2){
                servospate.setPosition(0.0); //pozitie transfer
            }*/
            if(sb1 && !sb2){
                servobara.setPosition(0);//deschis
            }else if(!sb1 && sb2){
                servobara.setPosition(0.4);//inchis
            }
            if(a1 && !b1) {
                claw2.setPosition(0.0);//gheara spate deschis
            } else if(!a1 && b1) {
                claw2.setPosition(0.45);//gheara spate inchis
            }

            if(su && !sd) {
                servobk.setPosition(0.1);//poz gheara spate transfer
            }else if (!su && sd){
                servobk.setPosition(0.6);//poz gheara spate punctat
            }

            if (a && !b) {
                claw.setPosition(0.35);//gheara preluare inchis
            } else if (!a && b) {
                claw.setPosition(-0.5);//gheara preluare deschis
            }
//
            if (upservo && !downservo) {
                bclaw.setPosition(0.02);//preluare
                servospate.setPosition(0.18);//poz preluare
            } else if (!upservo && downservo) {
                bclaw.setPosition(0.7);//transfer
                claw2.setPosition(0.0);//gheara spate deschis
                servospate.setPosition(0.02); //pozitie transfer

                servobk.setPosition(0.1);//poz gheara spate transfer
            }

            if (actup && !actdown) {
                actuator.setPower(0.4);//extindere intake
            }
            else if (!actup && actdown) {
                actuator.setPower(-0.6);//retragere intake
            } else if (!actup && !actdown)
                actuator.setPower(0);

            if(up>0 && down == 0) {
                sliderleft.setPower(0.4);
                sliderright.setPower(0.4);
            }
            else if (down < 0 && up == 0) {
                sliderleft.setPower(-0.8);
                sliderright.setPower(-0.8);
            }
            else {
                sliderleft.setPower(0);
                sliderright.setPower(0);
            }

            telemetry.addData("servospate: ", servospate.getPosition());
            telemetry.addData("Pivot pos: ", actuator.getCurrentPosition());
            telemetry.addData("bclaw: ", bclaw.getPosition());
            telemetry.addData("claw: ", claw.getPosition());
            telemetry.addData("claw2: ", claw2.getPosition());
            telemetry.addData("servobk: ", servobk.getPosition());
            telemetry.addData("left slide pts: ", sliderleft.getCurrentPosition());
            telemetry.addData("right slide pts: ", sliderright.getCurrentPosition());
            telemetry.update();
        }
    }
}
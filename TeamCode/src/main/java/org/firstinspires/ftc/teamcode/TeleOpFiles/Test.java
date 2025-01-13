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

import java.util.HashMap;

//ToDo: tune limits after motor replacement
@TeleOp
public class Test extends LinearOpMode {
    ServoImplEx bclaw = null;
    ServoImplEx claw = null;
    ServoImplEx claw2 = null;
    ServoImplEx servobk = null;
    ServoImplEx servospate = null;
    ServoImplEx servobara = null;
    DcMotorEx actuator = null;
    class choice {
        String f, s;
    }
    choice make_pair(String f, String s) {
        choice p = new choice();
        p.f = f;
        p.s = s;
        return p;
    }
    HashMap<Integer, choice> mp = new HashMap<Integer, choice>();
    HashMap<String, String> varnm = new HashMap<String, String>();
    String Lines = "";
    double pivcm = 8.722;
    void pivot(double cm, double pow) {
        int pts = (int)(cm * pivcm);
        actuator.setTargetPosition(pts);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setPower(pow);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        mp.put(0, make_pair("servobara", "servobara")); //servo_bara
        mp.put(1, make_pair("claw2", "servospate")); //claw2
        mp.put(2, make_pair("servospate", "clawrot")); //servospate
        mp.put(3, make_pair("bclaw", "claw_arm")); //bclaw
        mp.put(4, make_pair("servobk", "servobk")); //servobk
        mp.put(5, make_pair("claw", "claw")); //claw
        mp.put(6, make_pair("actuator", "actuator")); //Pivot

        actuator = hardwareMap.get(DcMotorEx.class, "Pivot");

        claw = hardwareMap.get(ServoImplEx.class, "claw"); //hook
        bclaw = hardwareMap.get(ServoImplEx.class, "bclaw");
        claw2 = hardwareMap.get(ServoImplEx.class, "claw2");
        servobk = hardwareMap.get(ServoImplEx.class, "servobk");
        servospate = hardwareMap.get(ServoImplEx.class, "servospate");
        servobara = hardwareMap.get(ServoImplEx.class, "servo_bara");

        boolean a1,b1,a, b, upservo, downservo, su,sd,sb1,sb2, conf;
        boolean actup, actdown, config = true, acted = false;
        int initenc = 0;

        waitForStart();

        if(isStopRequested()) return;

        for (int i = 0; i < 7 && !isStopRequested(); ++i) {
            Boolean first = false, second = false;
            telemetry.addData("Choose variable name (dpad_left/dpad_right)", mp.get(i).f + "/" + mp.get(i).s);
            telemetry.update();

            do {
                first = gamepad1.dpad_left;
                second = gamepad1.dpad_right;

                if (first)
                    varnm.put(mp.get(i).f, mp.get(i).f);
                else if (second)
                    varnm.put(mp.get(i).f, mp.get(i).s);
            } while (!first && !second);
            sleep(1000);
        }



        while (config && !isStopRequested()) {
            telemetry.addData("The tuning process", "");
            telemetry.update();

            boolean pressed = false;
            int enc = -1;
            String aux_command = "";

            a1 = gamepad2.a;
            b1 = gamepad2.b;
            conf = gamepad1.y;

            a = gamepad1.a; // servo close/open
            b = gamepad1.b;

            sb1= gamepad2.y;
            sb2 = gamepad2.x;

            actup = gamepad1.dpad_up;
            actdown = gamepad1.dpad_down;

            downservo = gamepad1.left_bumper;
            upservo = gamepad1.right_bumper;

            su = gamepad2.left_bumper;
            sd = gamepad2.right_bumper;

            if(sb1 && !sb2){
                servobara.setPosition(0);//deschis
                aux_command = varnm.get("servobara") + ".setPosition(0);\n";
                pressed = true;
            } else if(!sb1 && sb2){
                aux_command = varnm.get("servobara") + ".setPosition(0.4);\n";
                servobara.setPosition(0.4);//inchis
                pressed = true;
            }
            if(a1 && !b1) {
                claw2.setPosition(0.0);//gheara spate deschis
                aux_command = varnm.get("claw2") + ".setPosition(0);\n";
                pressed = true;
            } else if(!a1 && b1) {
                claw2.setPosition(0.45);//gheara spate inchis
                aux_command = varnm.get("claw2") + ".setPosition(0.45);\n";
                pressed = true;
            }

            if(su && !sd) {
                servobk.setPosition(0.1);//poz gheara spate transfer
                aux_command = varnm.get("servobk") + ".setPosition(0);\n";
                pressed = true;
            }else if (!su && sd){
                servobk.setPosition(0.6);//poz gheara spate punctat
                aux_command = varnm.get("servobk") + ".setPosition(0.6);\n";
                pressed = true;
            }

            if (a && !b) {
                claw.setPosition(0.35);//gheara preluare inchis
                aux_command = varnm.get("claw") + ".setPosition(0.35);\n";
                pressed = true;
            } else if (!a && b) {
                claw.setPosition(-0.5);//gheara preluare deschis
                aux_command = varnm.get("claw") + ".setPosition(0);\n";
                pressed = true;
            }
//
            if (upservo && !downservo) {
                bclaw.setPosition(0.02);//preluare
                servospate.setPosition(0.18);//poz preluare

                aux_command = varnm.get("bclaw") + ".setPosition(0.02);\n" +
                                varnm.get("servospate") + ".setPosition(0.18);\n";
                pressed = true;
            } else if (!upservo && downservo) {
                bclaw.setPosition(0.7);//transfer
                claw2.setPosition(0.0);//gheara spate deschis
                servospate.setPosition(0.02); //pozitie transfer
                servobk.setPosition(0.1);//poz gheara spate transfer

                aux_command = varnm.get("bclaw") + ".setPosition(0.7);\n" +
                        varnm.get("claw2") + ".setPosition(0);\n" +
                        varnm.get("servospate") + ".setPosition(0.02);\n" +
                        varnm.get("servobk") + ".setPosition(0.1);\n";
                pressed = true;
            }

            if (actup && !actdown) {
                actuator.setPower(0.4);//extindere intake
                enc = actuator.getCurrentPosition();
                acted = true;
            }
            else if (!actup && actdown) {
                actuator.setPower(-0.6);//retragere intake
                enc = actuator.getCurrentPosition();
                acted = true;
            } else if (!actup && !actdown && actuator.getPower() != 0) {
                actuator.setPower(0);
                telemetry.addData("Actuator pwr: ", actuator.getPower());
                telemetry.update();
                pressed = true;
                aux_command = "pivot(" + -actuator.getCurrentPosition()/pivcm + ");\n";
                acted = false;
            }

            while (pressed && !isStopRequested()) {
                telemetry.addData("Register command?(Y/N)(dpad_left/dpad_right): ", aux_command);
                telemetry.update();

                if (gamepad1.dpad_right) {
                    Lines += aux_command;
                    telemetry.addData("Commend status: ", "Registered");
                    telemetry.update();
                    sleep(1000);
                } else if (gamepad1.dpad_left) {
                    pressed = false;
                }
            }

            if (conf) {
                while (config && !isStopRequested()) {
                    telemetry.addData("Do you want to proceed?", "(Press again to confirm!)");
                    telemetry.update();

                    config = !conf;
                }
            }
        }

        while (!isStopRequested()) {
            telemetry.addData(Lines, "Code submitted successfully!");
            telemetry.update();
        }
    }
}

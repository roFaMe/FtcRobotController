package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Timer;

@TeleOp(name="TeleOP", group="Infinity")
//@Disabled
public class TeleOP extends LinearOpMode{
    //Таймер
    Timer time = new Timer();
    //Железо
    private DcMotor leftRear, rightRear, rightFront, leftFront, baraban;
    private Servo upDown, hook;

    //Переменные моторов

    private double zm1, zm2, zm3, zm4;
    int telescopePos;
    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free;
    private double a, turn, timesFUp, timesFHo;

    private double up = 0.63;
    private double ho = 0.8;
    private ElapsedTime runtime = new ElapsedTime();
    File telescopeFile = AppUtil.getInstance().getSettingsFile("telescopeFile.txt"); //Файл с позицией телескопа
    //Гироскоп

    //Инициализируем железо
    public void initC() {
        //Инициализация
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        baraban = hardwareMap.get(DcMotor.class, "baraban");

        upDown = hardwareMap.get(Servo.class, "upDown");
        hook = hardwareMap.get(Servo.class, "hook");

        upDown.setPosition(up);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        baraban.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baraban.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baraban.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread c;
            private boolean running;

            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    baraban.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    while (!isStopRequested() & opModeIsActive()) {

                        //ТЕЛЕЖКА


                        //Коэффицент скорости робота
                        if (gamepad1.left_trigger < 0.5) {
                            a = 0.7;
                        } else if (gamepad1.left_trigger > 0.5) {
                            a = 10;
                        }

                        //Поворот
                        turn = -gamepad1.right_stick_x;


                        //Мощность моторов тележки
                        zm1 = Range.clip((+gamepad1.left_stick_x - gamepad1.left_stick_y - turn) * a, -1, 1);
                        if (zm1 > -0.05 && zm1 < 0.05) {
                            zm1 = 0;
                        }

                        zm2 = Range.clip((+gamepad1.left_stick_x + gamepad1.left_stick_y - turn) * a, -1, 1);
                        if (zm2 > -0.05 && zm2 < 0.05) {
                            zm2 = 0;
                        }

                        zm3 = Range.clip((-gamepad1.left_stick_x - gamepad1.left_stick_y - turn) * a, -1, 1);
                        if (zm3 > -0.05 && zm3 < 0.05) {
                            zm3 = 0;
                        }

                        zm4 = Range.clip((-gamepad1.left_stick_x + gamepad1.left_stick_y - turn) * a, -1, 1);
                        if (zm4 > -0.05 && zm4 < 0.05) {
                            zm4 = 0;
                        }


                        //ТЕЛЕСКОП


                       baraban.setPower(-gamepad2.left_stick_y);


                        //Серваки

                        if (gamepad2.a && timesFHo == 0 && moment_diff_serv > 200) {
                            ho = 0.65;
                            timesFHo = 1;
                            last_moment_serv = runtime.milliseconds();
                        }

                        else if (gamepad2.a && timesFHo == 1 && moment_diff_serv > 200) {
                            ho = 0.8;
                            timesFHo = 0;
                            last_moment_serv = runtime.milliseconds();
                        }

                        if (gamepad2.x && timesFUp == 0 && moment_diff_serv > 200) {
                            up = 0.15;
                            timesFUp = 1;
                            last_moment_serv = runtime.milliseconds();
                        }

                        else if (gamepad2.x && timesFUp == 1 && moment_diff_serv > 200) {
                            up = 0.63;
                            timesFUp = 0;
                            last_moment_serv = runtime.milliseconds();
                        }


                        moment_diff_serv = runtime.milliseconds() - last_moment_serv;
                        moment_diff_switch = runtime.milliseconds() - last_moment_switch;

                    }
                } catch (Exception e) {
                    telemetry.addLine("Calc thread interrupted");
                    telemetry.update();
                }
            }
            public void start_c() {
                if (c == null) {
                    c = new Thread(this, "Calc thread");
                    c.start();
                }
            }
        }

        //Инициализация
        initC();

        waitForStart();

        //Запуск подпроцессов
        CalcThread C1 = new CalcThread();
        C1.start_c();

        //ОСНОВНАЯ ПРОГРАММА

        while(opModeIsActive() & !isStopRequested()) {

            leftFront.setPower(zm1);//слева спереди
            rightFront.setPower(zm2);//справа спереди
            leftRear.setPower(zm3);//слева сзади
            rightRear.setPower(zm4);//справа сздади

            upDown.setPosition(up);
            hook.setPosition(ho);
//            En3.setPower(lamp);

            telemetry.addData("Энкодер барабана", baraban.getCurrentPosition());
            telemetry.addData("Гейм2 - X", gamepad2.x);
            telemetry.addData("Гейм2 - A", gamepad2.a);
            telemetry.addData("Вверх-вниз", upDown.getPosition());
            telemetry.addData("Крючок", hook.getPosition());
            telemetry.update();

        }


        ReadWriteFile.writeFile(telescopeFile, Integer.toString(telescopePos));
    }
}

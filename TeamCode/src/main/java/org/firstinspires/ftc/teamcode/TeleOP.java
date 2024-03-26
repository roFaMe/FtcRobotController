package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotor leftRear, rightRear, rightFront, leftFront, baraban, arm, EnBar, leftSuck,rightSuck;
    private Servo upDown, hook_back,hook_front, plane;

    //Переменные моторов

    private double zm1, zm2, zm3, zm4;
    int telescopePos;
    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free;
    private double a, turn, timesFUp, timesFHoB, timesFHoF;
    double  ho_b = 0.2;//захват
    double ho_f = 0.5;
    double pl = 0.45;//начальное положение
    private double up = 0.0;
    private double speed = 0.65 ;

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

        leftSuck = hardwareMap.get(DcMotor.class, "leftSuck");
        rightSuck = hardwareMap.get(DcMotor.class, "rightSuck");

        baraban = hardwareMap.get(DcMotor.class, "EnX");
//        arm = hardwareMap.get(DcMotor.class, "arm");
//        EnBar = hardwareMap.get(DcMotor.class, "EnBar");

        upDown = hardwareMap.get(Servo.class, "upDown");
        hook_back = hardwareMap.get(Servo.class, "hook_back");
        hook_front = hardwareMap.get(Servo.class, "hook_front");
//        plane = hardwareMap.get(Servo.class, "plane");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSuck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSuck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread c;
            private boolean running;

            public void run() {
                telemetry.addLine("running");
                telemetry.update();

                try {
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    while (!isStopRequested() & opModeIsActive()) {

                        //ТЕЛЕЖКА


                        //Коэффицент скорости робота
                        if (gamepad1.left_trigger < 0.5) {
                            a = 1;
                        } else if (gamepad1.left_trigger > 0.5) {
                            a = 5;
                        }

                        //Поворот
                        turn = -gamepad1.right_stick_x;

                        //Мощность моторов тележки
                        zm1 = Range.clip((-gamepad1.left_stick_x + gamepad1.left_stick_y + turn) * a, -speed , speed );
                        if (zm1 > -0.05 && zm1 < 0.05) {
                            zm1 = 0;
                        }

                        zm2 = Range.clip((gamepad1.left_stick_x + gamepad1.left_stick_y - turn) * a, -speed , speed  );
                        if (zm2 > -0.05 && zm2 < 0.05) {
                            zm2 = 0;
                        }

                        zm3 = Range.clip((gamepad1.left_stick_x + gamepad1.left_stick_y + turn) * a, -speed, speed);
                        if (zm3 > -0.05 && zm3 < 0.05) {
                            zm3 = 0;
                        }

                        zm4 = Range.clip((-gamepad1.left_stick_x + gamepad1.left_stick_y - turn) * a, -speed, speed);
                        if (zm4 > -0.05 && zm4 < 0.05) {
                            zm4 = 0;
                        }

                        if (gamepad2.left_trigger > 0.05 ){
                            leftSuck.setPower(0.5);
                            rightSuck.setPower(-0.5);

                        }else{leftSuck.setPower(0);
                            rightSuck.setPower(0);}

                        if (gamepad2.right_trigger > 0.05 ){
                            leftSuck.setPower(-0.5);
                            rightSuck.setPower(0.5);

                        }else{leftSuck.setPower(0);
                            rightSuck.setPower(0);}

                        //ТЕЛЕСКОП

                        baraban.setPower(-gamepad2.left_stick_y);

                        //Серваки

                        if (gamepad2.a && timesFHoB == 0 && moment_diff_serv > 1000) {
                            hook_back.setPosition(0.2);
                            timesFHoB = 1;
                            last_moment_serv = runtime.milliseconds();
                        }

                        else if (gamepad2.a && timesFHoB == 1 && moment_diff_serv > 1000) {
                            hook_back.setPosition(0.5);
                            timesFHoB = 0;
                            last_moment_serv = runtime.milliseconds();
                        }

                        if (gamepad2.b && timesFHoF == 0 && moment_diff_serv > 500) {
                            hook_front.setPosition(0.2);
                            timesFHoF = 1;
                            last_moment_serv = runtime.milliseconds();
                        }

                        else if (gamepad2.b && timesFHoF == 1 && moment_diff_serv > 500) {
                            hook_front.setPosition(0.5);
                            timesFHoF = 0;
                            last_moment_serv = runtime.milliseconds();
                        }
//                        if (gamepad2.y && timesFHo == 0 && moment_diff_serv > 200) {
//                            ho = 0.65;
//                            timesFHo = 1;
//                            last_moment_serv = runtime.milliseconds();
//                        }
//
                        if (gamepad2.x && timesFUp == 0 && moment_diff_serv > 200) {
                            up = 0.3;
                            timesFUp = 1;
                            last_moment_serv = runtime.milliseconds();
                        }

                        else if (gamepad2.x && timesFUp == 1 && moment_diff_serv > 200) {
                            up = 0.75;
                            timesFUp = 0;
                            last_moment_serv = runtime.milliseconds();
                        }


                        moment_diff_serv = runtime.milliseconds() - last_moment_serv;
                        moment_diff_switch = runtime.milliseconds() - last_moment_switch;
//
//                        if(gamepad2.left_bumper){
//                            arm.setPower(1);
//                        }else{arm.setPower(0);}
//                        if(gamepad2.right_bumper){
//                            arm.setPower(-1);
//                        }else {arm.setPower(0);}
//
//                        if(gamepad2.back){
//                            pl = 0.15;
//                        }else{pl = 0.45;}

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
//            hook_front.setPosition(ho_f);
//            plane.setPosition(pl);
//
//            telemetry.addData("Энкодер барабана", -EnBar.getCurrentPosition());
//            telemetry.addData("Гейм2 - X", gamepad2.x);
//            telemetry.addData("Гейм2 - A", gamepad2.a);
//            telemetry.addData("Вверх-вниз", upDown.getPosition());
//            telemetry.addData("Крючок", hook.getPosition());
//            telemetry.addData("Самолёт", plane.getPosition());
            telemetry.addData("a", a);
            telemetry.addData("up_pos", upDown.getPosition());
            telemetry.addData("Lf", leftFront.getPower());
            telemetry.addData("Lr", leftRear.getPower());
            telemetry.addData("Rf", rightFront.getPower());
            telemetry.addData("Rr", rightRear.getPower());
            telemetry.addData("turn", turn);
            telemetry.update();

        }


        ReadWriteFile.writeFile(telescopeFile, Integer.toString(telescopePos));
    }
}

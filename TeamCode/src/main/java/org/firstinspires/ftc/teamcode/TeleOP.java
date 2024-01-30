
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.util.Timer;

@TeleOp(name="TeleOP", group="Infinity")
public class TeleOP {
//@Disabled
    public class TelepopRed extends LinearOpMode{
        //Таймер
        Timer time = new Timer();
        //Железо
        DcMotor leftBack, leftFront, rightFront = hardwareMap.get(DcMotor.class, "rightFront"), rightBack, baraban, hang;
        Servo grab, bibb_cock;
        private DigitalChannel touch;

        //Переменные моторов

        private double test =0;
        private double zm1, zm2, zm3, zm4, zm5;
        private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0;
        private double moment_diff_serv, moment_diff_switch, moment_diff_free;
        private boolean auto_mode = true, free_mode = false;
        private double a, turn;
        int telescopePos = 0;
        private ElapsedTime runtime = new ElapsedTime();

        private double lamp=0;
        private int height;
        File telescopeFile = AppUtil.getInstance().getSettingsFile("telescopeFile.txt"); //Файл с позицией телескопа
        private int svob=0;
        //Гироскоп

        //Инициализируем железо
        public void initC() {
            //Инициализация
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            baraban = hardwareMap.get(DcMotor.class, "baraban");
            hang = hardwareMap.get(DcMotor.class, "hang");
            

            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            baraban.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            


            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baraban.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            baraban.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            

            touch = hardwareMap.get(DigitalChannel.class, "touch");
            touch.setMode(DigitalChannel.Mode.INPUT);
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
                        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        baraban.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                        while (!isStopRequested() & opModeIsActive()) {
                            ////////////////////////ТЕЛЕЖКА//////////////////////////////////////////////

                            //Коэффицент скорости робота
                            if(gamepad1.left_trigger<0.5){
                                a=0.7;
                            }
                            else if(gamepad1.left_trigger>0.5){
                                a=10;
                            }
                            //Поворот
                            turn = -gamepad1.right_stick_x;


                            //Мощность моторов тележки
                            zm1 = Range.clip((-gamepad1.left_stick_x + gamepad1.left_stick_y - turn ) * a, -1, 1);
                            if (zm1 > -0.05 && zm1 < 0.05) {
                                zm1 = 0;
                            }

                            zm2 = Range.clip((-gamepad1.left_stick_x - gamepad1.left_stick_y - turn ) * a, -1, 1);
                            if (zm2 > -0.05 && zm2 < 0.05) {
                                zm2 = 0;
                            }

                            zm3 = Range.clip((gamepad1.left_stick_x - gamepad1.left_stick_y - turn ) * a, -1, 1);
                            if (zm3 > -0.05 && zm3 < 0.05) {
                                zm3 = 0;
                            }

                            zm4 = Range.clip((gamepad1.left_stick_x + gamepad1.left_stick_y - turn ) * a, -1, 1);
                            if (zm4 > -0.05 && zm4 < 0.05) {
                                zm4 = 0;
                            }

                            //ТЕЛЕСКОП
                            if(gamepad1.left_bumper==true){

                                if(gamepad1.left_trigger > 0.08){
                                    hang.setPower(gamepad1.left_trigger*100);
                                }

                                if(gamepad1.right_trigger>0.08){
                                    hang.setPower(gamepad1.right_trigger*-100);
                                }

                            }


                            else {
                                if(gamepad1.left_trigger > 0.08){
                                    hang.setPower(gamepad1.left_trigger*1);
                                }

                                if(gamepad1.right_trigger>0.08){
                                    hang.setPower(gamepad1.right_trigger*-1);
                                }
                            }

                            //вычисление задержки
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

                leftBack.setPower(zm1);//слева спереди
                leftFront.setPower(zm2);//справа спереди
                rightBack.setPower(zm3);//слева сзади
                rightFront.setPower(zm4);//справа сздади
                baraban.setPower(zm5);//барабан
                hang.setPower(zm5);//висеть


            }
            ReadWriteFile.writeFile(telescopeFile, Integer.toString(telescopePos));
        }
    }

}

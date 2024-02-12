package org.firstinspires.ftc.teamcode.Camera.Parents;

import android.graphics.Path;
import android.graphics.Region;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoMethods;
import org.firstinspires.ftc.teamcode.Direction;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous
public class Webcam extends LinearOpMode implements Direction {
    AutoMethods bot = new AutoMethods();
    protected OpenCvCamera webcam;
    protected DcMotor leftRear, rightRear, rightFront, leftFront, baraban, EnBar;
    protected Servo upDown, hook;
    protected ElapsedTime runtime = new ElapsedTime();
    protected BNO055IMU imu;
    protected Pipeline.Location location;
    public OpMode op;

    double runningtime  = 0;
    public String whatsAuto;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.initC(this);

        initCam(telemetry, this);
        camOpen();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        camClose();
        if(location == Pipeline.Location.LEFT){
            if(whatsAuto == "redLeft"){bot.drive(this, 0.5,LEFT);}

        }
        else if(location == Pipeline.Location.CENTER){
            bot.drive(this, 0.5,FORWARD);
        }
        else{
            bot.drive(this, 0.5,RIGHT);
        }



    }
    public void initCam(Telemetry telemetry, OpMode op){

    }
    public void camOpen() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public void camClose(){
        webcam.stopStreaming();
    }
    public void initC(OpMode op){
        this.op = op;

        //Инициализация
        leftFront = op.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = op.hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = op.hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = op.hardwareMap.get(DcMotor.class, "rightRear");
        baraban = op.hardwareMap.get(DcMotor.class, "baraban");

        EnBar = op.hardwareMap.get(DcMotor.class, "EnBar");

        upDown = op.hardwareMap.get(Servo.class, "upDown");
        hook = op.hardwareMap.get(Servo.class, "hook");

//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        baraban.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        EnBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        baraban.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        EnBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        baraban.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        EnBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void initIMU(OpMode op) {
        this.op = op;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void moveTele(int lift){
        if(lift != baraban.getCurrentPosition()){
            if(lift > 0){
                while(lift != baraban.getCurrentPosition()){
                    baraban.setPower(0.5);
                }baraban.setPower(0);
            }
            if(lift < 0){
                while(lift != baraban.getCurrentPosition()){
                    baraban.setPower(-0.5);
                }baraban.setPower(0);
            }
        }
    }
    public void motoStop(){
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }
    public void movement(String side, double time, OpMode op){
        this.op = op;
        runtime.reset();

        if(side.equals(FORWARD)){

            while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < time){

                leftFront.setPower(0.5);
                rightFront.setPower(0.5);
                leftRear.setPower(0.5);
                rightRear.setPower(0.5);

            }
            motoStop();

        }
        if(side.equals(LEFT)){
            while (time != runningtime && opModeIsActive() && !isStopRequested()){
                leftFront.setPower(-0.5);
                rightFront.setPower(0.5);
                leftRear.setPower(0.5);
                rightRear.setPower(-0.5);
            }
            runningtime = 0;
            motoStop();
        }
        if(side.equals(BACK)){
            while (time != runningtime && opModeIsActive() && !isStopRequested()){
                leftFront.setPower(-0.5);
                rightFront.setPower(-0.5);
                leftRear.setPower(-0.5);
                rightRear.setPower(-0.5);
            }
            runningtime = 0;
            motoStop();

        }
        if(side.equals(RIGHT)){
            while (time != runningtime && opModeIsActive() && !isStopRequested()){
                leftFront.setPower(0.5);
                rightFront.setPower(-0.5);
                leftRear.setPower(-0.5);
                rightRear.setPower(0.5);
            }
            runningtime = 0;
            motoStop();
        }

    }

}

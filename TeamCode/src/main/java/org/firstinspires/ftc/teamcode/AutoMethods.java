package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;


@Disabled
@Autonomous
public class AutoMethods extends LinearOpMode implements Direction{

    public OpenCvCamera webcam;
    public boolean camError = false;
    public ElapsedTime runtime = new ElapsedTime();
    public OpMode op;
    public int baza = 1;
    private BNO055IMU imu;
    private Orientation angles;


    //Железо

    private DcMotor leftRear, rightRear, rightFront, leftFront, baraban, EnBar, arm, EnYL, EnYR, EnX;
    public Servo upDown, hook;
    private int sleep = 350;
    @Override
    public void runOpMode() throws InterruptedException {
    }
    //Инициализируем гироскоп
    public void initIMU(OpMode op) {
        this.op = op;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //Инициализируем железо
    public void initC(OpMode op) {
        this.op = op;
        leftRear = op.hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = op.hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = op.hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = op.hardwareMap.get(DcMotor.class, "rightFront");
        baraban = op.hardwareMap.get(DcMotor.class, "baraban");

        hook = op.hardwareMap.get(Servo.class, "hook");
        upDown = op.hardwareMap.get(Servo.class, "upDown");

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        baraban.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baraban.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baraban.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void hookOpen(){
        hook.setPosition(0.55);
    }
    public void hookClose(){
        hook.setPosition(0.8);
    }

public void turn (double degrees, OpMode op, double timeout){
    this.op = op;

    EnYL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    EnYR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    double max_accel = 0.8;
    double countin_degrees =  (EnYL.getCurrentPosition() - EnYR.getCurrentPosition()) / 2.0;
    double countin_accel = (max_accel/Math.abs((countin_degrees - degrees)/900));
    runtime.reset();

    while (!isStopRequested() && !opModeIsActive() && degrees != countin_degrees && runtime.seconds() < timeout){
        if((countin_degrees - degrees)/900 * countin_accel > 0.26){
            leftFront.setPower((countin_degrees - degrees)/900 * countin_accel);
            leftRear.setPower((countin_degrees - degrees)/900 * countin_accel);
            rightFront.setPower((-countin_degrees + degrees)/900 * countin_accel);
            rightRear.setPower((-countin_degrees + degrees)/900 * countin_accel);
        }else {
            if(countin_degrees > 0){
                leftFront.setPower(-0.25);
                leftRear.setPower(-0.25);
                rightFront.setPower(0.25);
                rightRear.setPower(0.25);
            }else{
                leftFront.setPower(0.25);
                leftRear.setPower(0.25);
                rightFront.setPower(-0.25);
                rightRear.setPower(-0.25);
            }
        }


        op.telemetry.addData("countin_degrees", Math.abs((EnYL.getCurrentPosition() - EnYR.getCurrentPosition()) / 2.0));

        op.telemetry.update();
    }
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftRear.setPower(0);
    rightRear.setPower(0);
}

public void drive(int X, int Y, OpMode op, double timeout) {
    this.op = op;

    EnYL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    EnYR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    int X_rasst = X;
    int Y_rasst = Y;
    int zazor = 0;
    double spdX = 0, spdY = 0;
    double encX = EnX.getCurrentPosition();
    double encY = -(EnYL.getCurrentPosition() + EnYR.getCurrentPosition()) / 2.0;
    double ostX = X_rasst-encX;
    double ostY =Y_rasst-encY;
    double SquareGip = (ostX*ostX) + (ostY*ostY);
    double Gip = Math.sqrt(SquareGip);
    double turn = 0;

    //PID
    double P = 0;
    double I = 0;
    double D = 0;

    double Kp = 0.00025;
    double Ki = 0;
    double Kd = 0.0005;

    double last_time = 0;
    double last_gip = Gip;

    double PID = 0;

    runtime.reset();


    while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout && Gip > zazor) {

        P = Kp * Gip;

        I += Ki * Gip;

        if (runtime.milliseconds() - last_time > 50)
        {
            D = Kd * (Gip - last_gip);
            last_time = runtime.milliseconds();
            last_gip = Gip;
        }

        PID = P + I + D;

        encX = EnX.getCurrentPosition();
        encY = (EnYL.getCurrentPosition() + EnYR.getCurrentPosition()) / 2.;

        SquareGip = (ostX*ostX) + (ostY*ostY);
        Gip = Math.sqrt(SquareGip);

        ostX = X_rasst-encX;
        ostY = Y_rasst-encY;

        spdX = ostX / Gip;
        spdY = ostY / Gip;

        turn = (-EnYL.getCurrentPosition() + EnYR.getCurrentPosition()) / 800.0;

        leftFront.setPower((-spdX + spdY - turn) * PID);
        rightFront.setPower((-spdX - spdY - turn) * PID);
        leftRear.setPower((spdX - spdY - turn) * PID);
        rightRear.setPower((spdX + spdY - turn) * PID);

        op.telemetry.addData("spdX", spdX);
        op.telemetry.addData("spdY", spdY);
        op.telemetry.addData("Speed", PID);

        op.telemetry.addData("X", EnX.getCurrentPosition());
        op.telemetry.addData("Y", (EnYL.getCurrentPosition() + EnYR.getCurrentPosition()) / 2);

        op.telemetry.addData("Энкодер слева", EnYL.getCurrentPosition());
        op.telemetry.addData("Энкодер справ", EnYR.getCurrentPosition());


        op.telemetry.addData("encX", encX);
        op.telemetry.addData("encY", encY);
        op.telemetry.addData("ostX", ostX);
        op.telemetry.addData("ostY", ostY);
        op.telemetry.update();

    }
    leftFront.setPower(0);
    leftRear.setPower(0);
    rightRear.setPower(0);
    rightFront.setPower(0);
}
    public void drive_by_time ( OpMode op, double timeout, String loc) {
        this.op = op;

        runtime.reset();
        if(loc.equals(FORWARD)){
            while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout ) {
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            leftRear.setPower(0.5);
            rightRear.setPower(0.5);
        }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);}

        if(loc.equals(LEFT)){
            while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout ) {
                leftFront.setPower(-0.5);
                rightFront.setPower(0.5);
                leftRear.setPower(-0.5);
                rightRear.setPower(0.5);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);}

        if(loc.equals(RIGHT)){
            while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout ) {
                leftFront.setPower(0.5);
                rightFront.setPower(-0.5);
                leftRear.setPower(0.5);
                rightRear.setPower(-0.5);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);}


        if(loc.equals(BACK)){
            while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout ) {
                leftFront.setPower(-0.5);
                rightFront.setPower(-0.5);
                leftRear.setPower(-0.5);
                rightRear.setPower(-0.5);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);}
    }

    public void Telescope (int number){
        while(!opModeIsActive() && !isStopRequested() && baraban.getCurrentPosition() != number){
            if (number - baraban.getCurrentPosition() > 10) {
                baraban.setPower(-0.75);
            }
            else if (number - baraban.getCurrentPosition() < -10) {
                baraban.setPower(0.5);
            }
            else {
                baraban.setPower(-0.05);
                break;
            }
        }
    }
}
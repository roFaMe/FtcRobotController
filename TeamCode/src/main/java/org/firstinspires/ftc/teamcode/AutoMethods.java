package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.util.ArrayList;


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

    private DcMotor leftRear, rightRear, rightFront, leftFront, baraban, EnBar, arm;
    private Servo upDown, hook;
    private int sleep = 350;
    public  enum  Direction{
        forward,
        back,
        left,
        right
    }
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

/*    public void camStart(OpMode op) {
        try {
            this.op = op;
            int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.openCameraDevice();
            webcam.setPipeline(new Detector(op.telemetry));
            webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        }catch (OpenCvCameraException e){camError = true;}
        catch (NullPointerException e2){camError = true;}
    }*/

    public void camStop() {
        if (!camError) {webcam.stopStreaming();}
    }
//    public void getPos() {
//
//        if (!camError) {
//            switch (Detector.getLocation()) {
//                case BLUE:
//                    baza = 1;
//                    break;
//                case YELLOW:
//                    baza = 2;
//                    break;
//                case STRIPES:
//                    baza = 3;
//                    break;
//            }
//        }
//
//    }
    public void drive ( OpMode op, double timeout) {
        this.op = op;

        runtime.reset();

        while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout ) {
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            leftRear.setPower(0.5);
            rightRear.setPower(0.5);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

//    public void Telescope (int number){
//        while(!opModeIsActive() && !isStopRequested() && m5.getCurrentPosition() != number){
//            if (number - m5.getCurrentPosition() > 10) {
//                m5.setPower(-0.75);
//            }
//            else if (number - m5.getCurrentPosition() < -10) {
//                m5.setPower(0.5);
//            }
//            else {
//                m5.setPower(-0.05);
//                break;
//            }
//        }
//    }
}
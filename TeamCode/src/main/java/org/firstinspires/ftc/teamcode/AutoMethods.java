package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.io.File;
import java.text.DecimalFormat;
import java.util.Locale;


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

    public DcMotor leftRear, rightRear, rightFront, leftFront, baraban, EnBar, arm, EnX;
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
        EnX = op.hardwareMap.get(DcMotor.class, "EnX");

//        baraban = op.hardwareMap.get(DcMotor.class, "baraban");
//
//        hook = op.hardwareMap.get(Servo.class, "hook");
//        upDown = op.hardwareMap.get(Servo.class, "upDown");

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        baraban.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EnX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        baraban.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EnX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void hookOpen(){
        hook.setPosition(0.55);
    }
    public void hookClose(){
        hook.setPosition(0.8);
    }

   public void passive (OpMode op){
        this.op = op;
       while (!isStopRequested() && !opModeIsActive() ) {
           op.telemetry.addData("СлСп", leftFront.getCurrentPosition());
           op.telemetry.addData("СзСл", leftRear.getCurrentPosition());
           op.telemetry.addData("СпрСп", rightFront.getCurrentPosition());
           op.telemetry.addData("СпрСз", rightRear.getCurrentPosition());
           op.telemetry.update();
       }
    }
    String formatAngle(AngleUnit
                               angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
public void turn ( OpMode op,double degrees, double timeout) {
    this.op = op;

    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    EnX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    EnX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    //всё указанно в см
    double distance_to_center_wheel_enX = 14.95;
    double distance_to_center_wheel = Math.sqrt((11.4 * 11.4) + (14.45 * 14.45));

    double wide_robot_walls = 36.9;
    double wide_robot_up = 37.8;

    double XY_centr[] = {wide_robot_up / 2, wide_robot_walls / 2};

    double diam_enX = 9;
    double diam_wheel = 10;

    double lengh_round_enX = Math.round(diam_enX * 3.14);
    double lengh_round_wheel = (diam_wheel * 3.14);

    double lengh_of_robot_round_by_wheel = Math.round(2 * distance_to_center_wheel * 3.14);
    double lengh_of_robot_round_by_enX = Math.round(2 * distance_to_center_wheel_enX * 3.14);

    double tpCM_enX = Math.floor(1600 / lengh_of_robot_round_by_enX);
    double tpCM_wheel = (145.1 * 4 / lengh_of_robot_round_by_wheel);

    double countin_CM_by_enX = 0;
    double countin_CM_by_wheel = 0;

    double rounds_by_enX = lengh_of_robot_round_by_enX / lengh_round_enX;
    double rounds_by_wheel = lengh_of_robot_round_by_wheel / lengh_round_wheel;

    double DIST_enX = (lengh_of_robot_round_by_enX / (360 / degrees) );
    double DIST_wheel = (lengh_of_robot_round_by_wheel / (360 / degrees));

    double max_accel = 0.5;
    double countin_degrees_1 = 0;
    double countin_degrees_2 = 0;
    double countin_accel_one = (Math.abs(max_accel) / Math.abs((countin_degrees_1 - degrees)));
    double countin_accel_two = (Math.abs(max_accel) / Math.abs((-countin_degrees_1 + degrees)));

    double countin_accel_one_2 = (Math.abs(max_accel) / Math.abs((countin_degrees_2 - degrees)));
    double countin_accel_two_2 = (Math.abs(max_accel) / Math.abs((-countin_degrees_2 + degrees)));

    double enX = EnX.getCurrentPosition();
    double enLY = (leftFront.getCurrentPosition() - leftRear.getCurrentPosition())/2.0;
    double enRY = (rightFront.getCurrentPosition() + rightRear.getCurrentPosition())/2.0;
    double vyr = enRY - enLY;
    double enY = (enRY-enLY)/2;

    double countin_accel_1 = (Math.abs(max_accel) / Math.abs((countin_CM_by_enX - DIST_enX)));
    double countin_accel_2 = (Math.abs(max_accel) / Math.abs((-countin_CM_by_enX + DIST_enX)));

    double countin_degrees = Math.ceil(((((-leftRear.getCurrentPosition() + leftFront.getCurrentPosition()) / 2.0
            - (rightFront.getCurrentPosition() + rightRear.getCurrentPosition()) / 2.0) / 4.0 * 0.745)));
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double max_speed = 0.4;
    double ost_angles =  degrees - Math.abs(angles.firstAngle);
    double v1 = Range.clip((countin_CM_by_enX -  ost_angles - DIST_enX) / degrees,-max_speed,max_speed);
    double v2 = Range.clip((-countin_CM_by_enX + ost_angles + DIST_enX) / degrees,-max_speed,max_speed);
    runtime.reset();


//    while ((!isStopRequested() && !opModeIsActive() && degrees != countin_degrees_1 && runtime.seconds() < timeout
//            && (leftRear.getPower() < 0.15 || leftRear.getPower() > -0.15 ))){
//
//        imu_pos = Math.ceil(Math.abs(imu.getAngularOrientation().firstAngle * 57.7));
//
//        countin_degrees_1 =  Math.ceil(((((-leftRear.getCurrentPosition() + leftFront.getCurrentPosition())/2.0
//                - (rightFront.getCurrentPosition() + rightRear.getCurrentPosition())/2.0)/4.0  * 0.745)));
//
//        enX = EnX.getCurrentPosition();
//
//        countin_degrees_2 = enX * 145.1 / 1600;
//
////    leftFront.setPower(Range.clip(((countin_degrees_1 - degrees) * countin_accel_one ) , -1, 1));
////    leftRear.setPower(Range.clip(((countin_degrees_1 - degrees)  * countin_accel_one ), -1, 1));
////    rightFront.setPower(Range.clip(((-countin_degrees_1 + degrees) * countin_accel_two), -1, 1));
////    rightRear.setPower(Range.clip(((-countin_degrees_1 + degrees) * countin_accel_two ), -1, 1));
//
////        leftFront.setPower(Range.clip(((countin_degrees_2 - degrees) * countin_accel_one_2 ) , -1, 1));
////        leftRear.setPower(Range.clip(((countin_degrees_2 - degrees)  * countin_accel_one_2 ), -1, 1));
////        rightFront.setPower(Range.clip(((-countin_degrees_2 + degrees) * countin_accel_two_2), -1, 1));
////        rightRear.setPower(Range.clip(((-countin_degrees_2 + degrees) * countin_accel_two_2 ), -1, 1));
//
//        op.telemetry.addData("lengh_of_robot_round_by_wheel", lengh_of_robot_round_by_wheel);
//        op.telemetry.addData("lengh_of_robot_round_by_enX", lengh_of_robot_round_by_enX);
//
//        op.telemetry.addData("lengh_round_enX", lengh_round_enX);
//        op.telemetry.addData("lengh_round_wheel", lengh_round_wheel);
//
//        op.telemetry.addData("tpCM_enX",Math.ceil(tpCM_enX));
//        op.telemetry.addData("tpCM_wheel", Math.ceil(tpCM_wheel));
//
//        op.telemetry.addData("lFpos", leftFront.getCurrentPosition());
//        op.telemetry.addData("lRpos", -leftRear.getCurrentPosition());
//        op.telemetry.addData("rFpos", rightFront.getCurrentPosition());
//        op.telemetry.addData("rRpos", rightRear.getCurrentPosition());
//
//
//        op.telemetry.update();
//
//    }
//    while (!isStopRequested() && !opModeIsActive() && countin_CM_by_wheel < (DIST_wheel)
//            || countin_CM_by_wheel > (-DIST_wheel) && !isStopRequested() && !opModeIsActive()) {
//        double a = 0.6;
//
////        enX = EnX.getCurrentPosition();
//        countin_CM_by_wheel = Math.floor(((((-leftRear.getCurrentPosition() + leftFront.getCurrentPosition()) / 2.0
//                - (rightFront.getCurrentPosition() + rightRear.getCurrentPosition()) / 2.0) / 4.0 * 0.745)) / tpCM_wheel);
//        countin_CM_by_enX = Math.floor(enX / tpCM_enX);
//
//        double v1 = (Range.clip(((countin_CM_by_wheel - (DIST_wheel))), -a, a));
//        double v2 = (Range.clip(((-countin_CM_by_wheel + (DIST_wheel))), -a, a));
//
//        leftFront.setPower(v1);
//        leftRear.setPower(v1);
//        rightFront.setPower(v2);
//        rightRear.setPower(v2);
//
//        op.telemetry.addData("rounds_by_wheel", rounds_by_wheel);
//
//        op.telemetry.addData("DIST", DIST_wheel);
//        op.telemetry.addData("countin_CM_by_wheel", countin_CM_by_wheel);
//        op.telemetry.addData("tpCM_enX", (tpCM_enX));
//        op.telemetry.addData("tpCM_wheel", (tpCM_wheel));
//
//        op.telemetry.addData("enX", enX);
//
//        op.telemetry.addData("lengh_of_robot_round_by_wheel", lengh_of_robot_round_by_wheel);
//        op.telemetry.addData("lengh_of_robot_round_by_enX", lengh_of_robot_round_by_enX);
//
//        op.telemetry.addData("lengh_round_enX", lengh_round_enX);
//        op.telemetry.addData("lengh_round_wheel", lengh_round_wheel);
//
//
//        op.telemetry.addData("lFpos", leftFront.getCurrentPosition());
//        op.telemetry.addData("lRpos", -leftRear.getCurrentPosition());
//        op.telemetry.addData("rFpos", rightFront.getCurrentPosition());
//        op.telemetry.addData("rRpos", rightRear.getCurrentPosition());
//
//        op.telemetry.addData("lFPOW", leftFront.getPower());
//        op.telemetry.addData("lRPOW", leftRear.getPower());
//        op.telemetry.addData("rFPOW", rightFront.getPower());
//        op.telemetry.addData("rRPOW", rightRear.getPower());
//
//        op.telemetry.update();
//    }
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    while (!isStopRequested() && !opModeIsActive()  && runtime.seconds() < timeout) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        enLY = (leftFront.getCurrentPosition() - leftRear.getCurrentPosition())/2.0;
        enRY = (rightFront.getCurrentPosition() + rightRear.getCurrentPosition())/2.0;
        enX = EnX.getCurrentPosition();
        vyr = (Math.abs(enLY) - Math.abs(enRY))/100.0;
        enY = Math.abs((enRY-enLY)/2);
        tpCM_enX = Math.floor((1600/lengh_of_robot_round_by_enX * 4.0));
        tpCM_wheel = Math.floor((145.1/ lengh_of_robot_round_by_wheel * 4.0));
        countin_CM_by_enX = enX/tpCM_enX;
        countin_CM_by_wheel = enY/tpCM_wheel;

        ost_angles =  degrees - (Math.abs(angles.firstAngle));

        if(Math.abs(v1) >= 0.15  ){
            v1 = Range.clip((countin_CM_by_enX - DIST_enX - vyr -  ost_angles ) / (degrees),-max_speed,max_speed);
            v2 = Range.clip((-countin_CM_by_enX + DIST_enX + vyr + ost_angles ) / (degrees),-max_speed,max_speed);
        }else{
            v1 = Range.clip((countin_CM_by_enX - DIST_enX - vyr -  ost_angles ) * 15.0 / (degrees) ,-max_speed,max_speed);
            v2 = Range.clip((-countin_CM_by_enX + DIST_enX + vyr + ost_angles ) * 15.0 / (degrees) ,-max_speed,max_speed);
        }

        leftFront.setPower(v1);
        leftRear.setPower(v1);
        rightFront.setPower(v2);
        rightRear.setPower(v2);

        op.telemetry.addData("lFPOW", leftFront.getPower());
        op.telemetry.addData("lRPOW", leftRear.getPower());
        op.telemetry.addData("rFPOW", rightFront.getPower());
        op.telemetry.addData("rRPOW", rightRear.getPower());
        op.telemetry.addData("DIST_enX", DIST_enX);
        op.telemetry.addData("DIST_wheel", DIST_wheel);
        op.telemetry.addData("lengh_of_robot_round_by_enX", lengh_of_robot_round_by_enX);
        op.telemetry.addData("lengh_of_robot_round_by_wheel", lengh_of_robot_round_by_wheel);
        op.telemetry.addData("countin_CM_by_enX", countin_CM_by_enX);
        op.telemetry.addData("countin_CM_by_wheel", countin_CM_by_wheel);
        op.telemetry.addData("vyr", vyr);
        op.telemetry.addData("enY",enY );
        op.telemetry.addData("enLY", enLY);
        op.telemetry.addData("enRY",enRY);
        op.telemetry.addData("enX",enX);
        op.telemetry.addData("angles",angles);
        op.telemetry.addData("getSystemStatus",imu.getSystemStatus().toShortString());
        op.telemetry.addData("getCalibrationStatus",imu.getCalibrationStatus().toString());
        op.telemetry.addData("firstAngle",formatAngle(angles.angleUnit, angles.firstAngle));
        op.telemetry.addData("secondAngle",formatAngle(angles.angleUnit, angles.secondAngle));
        op.telemetry.addData("thirdAngle",formatAngle(angles.angleUnit, angles.thirdAngle));

        op.telemetry.update();
    }
    leftFront.setPower(0.0);
    rightFront.setPower(0.0);
    leftRear.setPower(0.0);
    rightRear.setPower(0.0);
}

public void drive( OpMode op,double X, double Y, double timeout) {
    this.op = op;
    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    EnX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    EnX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    double enX = -EnX.getCurrentPosition();
    double enLY = (leftFront.getCurrentPosition() - leftRear.getCurrentPosition())/2.0;
    double enRY = (rightFront.getCurrentPosition() + rightRear.getCurrentPosition())/2.0;
    double vyr_x = (enLY - enRY) * (1600/145.1);
    double vyr_y = enX;
    double enY = (enRY-enLY)/2;
    double ostX = Math.abs(X) - enX;
    double ostY = Math.abs(Y) - enY;
    double gip = Math.sqrt((ostX * ostX) + (ostY * ostY));
    double spdX = enX-X;
    double spdY = -enY + Y;
    double max_speed = 1;
    double countin_accel_one = Math.abs((max_speed/(spdX - spdY)));
    double countin_accel_two = Math.abs((max_speed/(-spdX - spdY)));
    runtime.reset();

    while (!isStopRequested() && !opModeIsActive() && Math.abs((spdX + spdY + vyr_x) /(Math.abs(X) + Math.abs(Y))) >= 0.14) {
        enX = -EnX.getCurrentPosition();
        enLY = ((leftFront.getCurrentPosition() - leftRear.getCurrentPosition())/2.0) * (1600/145.1);
        enRY = ((rightFront.getCurrentPosition() + rightRear.getCurrentPosition())/2.0) * (1600/145.1);
        vyr_x = (enLY - enRY) * (1600/145.1);
        enY = (enRY + enLY)/2 * (1600/145.1);

         ostX = Math.abs(X) - enX;
         ostY = Math.abs(Y) - enY;

         gip = Math.sqrt((ostX * ostX) + (ostY * ostY));

         spdX = enX - X;

         spdY = enY - Y;

        leftFront.setPower(Range.clip((spdX + spdY + vyr_x )/(Math.abs(X) + Math.abs(Y)) , -max_speed, max_speed));
        rightFront.setPower(Range.clip((-spdX + spdY - vyr_x )/(Math.abs(X) + Math.abs(Y) ), -max_speed, max_speed));
        leftRear.setPower(Range.clip((-spdX + spdY + vyr_x )/(Math.abs(X) + Math.abs(Y) ), -max_speed, max_speed));
        rightRear.setPower(Range.clip((spdX + spdY - vyr_x)/(Math.abs(X) + Math.abs(Y)) , -max_speed, max_speed));

        op.telemetry.addData("lFPOW", leftFront.getPower());
        op.telemetry.addData("lRPOW", leftRear.getPower());
        op.telemetry.addData("rFPOW", rightFront.getPower());
        op.telemetry.addData("rRPOW", rightRear.getPower());

        op.telemetry.addData("enLY", enLY);
        op.telemetry.addData("enRY", enRY);

        op.telemetry.addData("vyr", vyr_x);
        op.telemetry.addData("enY", enY);
        op.telemetry.addData("enX", enX);

        op.telemetry.addData("countin_accel_one", countin_accel_one);
        op.telemetry.addData("countin_accel_two", countin_accel_two);

        op.telemetry.addData("spdX", spdX);
        op.telemetry.addData("spdY", spdY);

        op.telemetry.addData("gip", gip);

        op.telemetry.update();
    }
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftRear.setPower(0);
    rightRear.setPower(0);
}
    public void drive_by_time ( OpMode op, double timeout, String loc, double X ,double Y) {
        this.op = op;
        double enX = Math.abs(EnX.getCurrentPosition());
    double enY = Math.abs((Math.ceil((rightFront.getCurrentPosition() +  (leftFront.getCurrentPosition())
            + rightRear.getCurrentPosition() - leftRear.getCurrentPosition())/4) * (1600 /145.1)));
        runtime.reset();
        if(loc.equals(FORWARD)){
            while ((!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout)  ) {

            leftFront.setPower(-0.5);
            rightFront.setPower(-0.5);
            leftRear.setPower(-0.5);
            rightRear.setPower(-0.5);
                        op.telemetry.addData("enY", enY);
                        op.telemetry.update();
        }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);}

        if(loc.equals(LEFT)){
            while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout ) {
                leftFront.setPower(0.5);
                rightFront.setPower(-0.5);
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
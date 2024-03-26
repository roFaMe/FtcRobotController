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
    private BNO055IMU imu;
    private Orientation angles;

    //Железо

    public DcMotor leftRear, rightRear, rightFront, leftFront, baraban, EnBar, arm, EnX, EnLY, EnRY;
    public Servo upDown, hook_front, hook_back;
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
        baraban = op.hardwareMap.get(DcMotor.class, "baraban");

        hook_front = op.hardwareMap.get(Servo.class, "hook_front");
        hook_back = op.hardwareMap.get(Servo.class, "hook_back");
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
        EnX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baraban.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EnX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baraban.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void hookOpenFront(){
        hook_front.setPosition(0.3);
    }
    public void hookCloseFront(){
        hook_front.setPosition(0.0);
    }

    public void hookOpenBack(){
        runtime.reset();
        while (runtime.milliseconds() < 700){hook_back.setPosition(0.3);}
    }
    public void hookCloseBack(){
        hook_back.setPosition(0.0);
    }
    public void upDownUP(){
        runtime.reset();
        while (runtime.milliseconds() < 700) {upDown.setPosition(1);}

    }
    public void upDownDown(){
        upDown.setPosition(0.6);
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
    double distance_to_center_enX = 12;
    double distance_to_center_wheel = 18.4;

    double diam_enX = 4.8;
    double diam_wheel = 10;

    double lengh_round_enX = Math.round(diam_enX * 3.14);
    double lengh_round_wheel = (diam_wheel * 3.14);

    double lengh_of_robot_round_by_wheel = (2 * distance_to_center_wheel * 3.14);
    double lengh_of_robot_round_by_enX = (2 * distance_to_center_enX * 3.14);

    double raznost_rounds = lengh_of_robot_round_by_enX/lengh_of_robot_round_by_wheel;

    double countin_degrees = 0;
    double enX = EnX.getCurrentPosition();
    double enLY = (leftFront.getCurrentPosition() - leftRear.getCurrentPosition())/2.0;
    double enRY = (rightFront.getCurrentPosition() + rightRear.getCurrentPosition())/2.0;
    double vyr = enRY - enLY;
    double enY = (enRY - enLY)/2;
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double max_speed = 1;
    double ost_angles =  degrees - Math.abs(angles.firstAngle);
    double vyr_ug =  (-angles.firstAngle / degrees);
    double spd = countin_degrees - degrees;
    double v1 = Range.clip((spd + vyr_ug ) ,-max_speed,max_speed);
    double v2 = Range.clip((-spd - vyr_ug )  ,-max_speed,max_speed);
    double ticks_per_degrees_enY = 1.56;
    double rasst = degrees * ticks_per_degrees_enY * 360/degrees * (1.53);

    runtime.reset();

    while (!isStopRequested() && !opModeIsActive()  && runtime.seconds() < timeout  ) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        enLY = -(leftFront.getCurrentPosition() + leftRear.getCurrentPosition())/2.0;
        enRY = -(rightFront.getCurrentPosition() + rightRear.getCurrentPosition())/2.0;
        enX = -EnX.getCurrentPosition();
        enY = -(enRY-enLY)/2.0;

        spd = enY/rasst;
        vyr = -angles.firstAngle/950.0;

        if(degrees < 0){
            if(Math.abs(spd - max_speed + vyr ) > 0.087){
                leftFront.setPower(Range.clip((spd + max_speed + vyr ) ,-max_speed ,max_speed ));
                leftRear.setPower(Range.clip((spd + max_speed + vyr) ,-max_speed,max_speed ));
                rightFront.setPower(Range.clip(( -spd - max_speed - vyr) ,-max_speed,max_speed));
                rightRear.setPower(Range.clip(( -spd - max_speed - vyr) ,-max_speed,max_speed));
            }else{
                break;}
        }else{
            if(Math.abs(spd - max_speed + vyr ) > 0.087){
                leftFront.setPower(Range.clip((spd - max_speed + vyr ) ,-max_speed ,max_speed ));
                leftRear.setPower(Range.clip((spd - max_speed + vyr) ,-max_speed,max_speed ));
                rightFront.setPower(Range.clip(( -spd + max_speed - vyr) ,-max_speed,max_speed));
                rightRear.setPower(Range.clip(( -spd + max_speed - vyr) ,-max_speed,max_speed));
            }else{
                break;}
        }

        op.telemetry.addData("spd", spd);
        op.telemetry.addData("rasst", rasst);

        op.telemetry.addData("lFPOW", leftFront.getPower());
        op.telemetry.addData("lRPOW", leftRear.getPower());
        op.telemetry.addData("rFPOW", rightFront.getPower());
        op.telemetry.addData("rRPOW", rightRear.getPower());

        op.telemetry.addData("angles", -angles.firstAngle);
        op.telemetry.addData("lFPOS", -leftFront.getCurrentPosition());
        op.telemetry.addData("lRPOS", -leftRear.getCurrentPosition());
        op.telemetry.addData("rFPOS", -rightFront.getCurrentPosition());
        op.telemetry.addData("rRPOS", -rightRear.getCurrentPosition());

        op.telemetry.addData("vyr", vyr);

        op.telemetry.addData("enY",enY );
        op.telemetry.addData("enX",enX );

        op.telemetry.addData("enLY", enLY);
        op.telemetry.addData("enRY",enRY );

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

    double enX = EnX.getCurrentPosition();
    double enLY = (-leftFront.getCurrentPosition() - leftRear.getCurrentPosition()) / 2.0;
    double enRY = (-rightFront.getCurrentPosition() - rightRear.getCurrentPosition()) / 2.0;
    double enY = (enRY - enLY) / 2;

    double spdX = enX - X;
    double spdY = -enY + Y;

    double max_speed = 1;
    double countin_degrees = (enLY - enRY) / 4.0;

    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double vyr = 0;
    runtime.reset();

    while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        enX = EnX.getCurrentPosition();
        enLY = ((-leftFront.getCurrentPosition() - leftRear.getCurrentPosition()) / 2.0);
        enRY = ((-rightFront.getCurrentPosition() - rightRear.getCurrentPosition()) / 2.0);
        enY = (enRY + enLY) / 2;

        if (X != 0 && Y == 0) {
            vyr = -angles.firstAngle / 40.0;
        } else {
            vyr = -angles.firstAngle / 70.0;
        }
        spdX = enX / X;
        spdY = Math.abs(enY) / Y;


        if (Math.abs((spdY - max_speed) + vyr) > 0.087) {
            if (Y != 0 && X == 0) {
                if(Y > 0 ){
                    leftFront.setPower(Range.clip(((spdY - max_speed) + vyr), -max_speed, max_speed));
                    rightFront.setPower(Range.clip(((spdY - max_speed) - vyr), -max_speed, max_speed));
                    leftRear.setPower(Range.clip(((spdY - max_speed) + vyr), -max_speed, max_speed));
                    rightRear.setPower(Range.clip(((spdY - max_speed) - vyr), -max_speed, max_speed));
                }else{
                    leftFront.setPower(Range.clip(((spdY + max_speed) + vyr), -max_speed, max_speed));
                    rightFront.setPower(Range.clip(((spdY + max_speed) - vyr), -max_speed, max_speed));
                    leftRear.setPower(Range.clip(((spdY + max_speed) + vyr), -max_speed, max_speed));
                    rightRear.setPower(Range.clip(((spdY + max_speed) - vyr), -max_speed, max_speed));
                }

            } else if (X != 0 && Y == 0) {
                if(X > 0){
                    leftFront.setPower(Range.clip(((spdX - max_speed) + vyr), -max_speed, max_speed));
                    rightFront.setPower(Range.clip(((-spdX + max_speed) - vyr), -max_speed, max_speed));
                    leftRear.setPower(Range.clip(((-spdX + max_speed) + vyr), -max_speed, max_speed));
                    rightRear.setPower(Range.clip(((spdX - max_speed - vyr)), -max_speed, max_speed));
                }else{
                    leftFront.setPower(Range.clip(((-spdX - max_speed) + vyr), -max_speed, max_speed));
                    rightFront.setPower(Range.clip(((spdX + max_speed) - vyr), -max_speed, max_speed));
                    leftRear.setPower(Range.clip(((spdX + max_speed) + vyr), -max_speed, max_speed));
                    rightRear.setPower(Range.clip(((-spdX - max_speed - vyr)), -max_speed, max_speed));
                }

            } else {
                leftFront.setPower(Range.clip(((spdY - max_speed) + (spdX - max_speed) + vyr), -max_speed, max_speed));
                rightFront.setPower(Range.clip(((-spdY + max_speed) + (-spdX + max_speed) - vyr), -max_speed, max_speed));
                leftRear.setPower(Range.clip(((-spdY + max_speed) + (-spdX + max_speed) + vyr), -max_speed, max_speed));
                rightRear.setPower(Range.clip(((spdY - max_speed) + (spdX - max_speed) - vyr), -max_speed, max_speed));
            }
        }else{
            break;
        }


        op.telemetry.addData("angles.firstAngle", -angles.firstAngle);
        op.telemetry.addData("vyr", vyr);

        op.telemetry.addData("lFPOW", leftFront.getPower());
        op.telemetry.addData("lRPOW", leftRear.getPower());
        op.telemetry.addData("rFPOW", rightFront.getPower());
        op.telemetry.addData("rRPOW", rightRear.getPower());

        op.telemetry.addData("lFPOS", -leftFront.getCurrentPosition());
        op.telemetry.addData("lRPOS", -leftRear.getCurrentPosition());
        op.telemetry.addData("rFPOS", -rightFront.getCurrentPosition());
        op.telemetry.addData("rRPOS", -rightRear.getCurrentPosition());

        op.telemetry.addData("enLY", enLY);
        op.telemetry.addData("enRY", enRY);
        op.telemetry.addData("enY", enY);
        op.telemetry.addData("enX", enX);

        op.telemetry.addData("spdX", spdX);
        op.telemetry.addData("spdY", spdY);

        op.telemetry.update();

    }

    leftFront.setPower(0);
    rightFront.setPower(0);
    leftRear.setPower(0);
    rightRear.setPower(0);
}

public  void  left (OpMode op, double timeout){
        runtime.reset();
    while(!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout ){
        leftFront.setPower(1);
        rightFront.setPower(-1);
        leftRear.setPower(-1);
        rightRear.setPower(1);
    }
}
    public void telescope (OpMode op, int number, double timeout){
        baraban.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baraban.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.op = op;
        runtime.reset();

        while(!opModeIsActive() && !isStopRequested() && baraban.getCurrentPosition() != number ){
            if (number - baraban.getCurrentPosition() > 10) {
                baraban.setPower(0.75);
            }
            else if (number - baraban.getCurrentPosition() < -10) {
                baraban.setPower(-0.5);
            }
            else {
                baraban.setPower(0.1);
                break;
            }
        }
    }
}
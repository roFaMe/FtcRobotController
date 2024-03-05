package org.firstinspires.ftc.teamcode.Camera.Parents;

import android.graphics.Path;
import android.graphics.Region;
import android.util.Size;

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoMethods;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Disabled
@Autonomous
public class Webcam extends LinearOpMode implements Direction {
    AutoMethods bot = new AutoMethods();

    protected OpenCvCamera webcam;
    protected ElapsedTime runtime = new ElapsedTime();
    protected Pipeline.Location location;
    public OpMode op;

    double runningtime  = 0;
    public String whatsAuto;

    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .build();

    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480))
            .build();


    @Override
    public void runOpMode() throws InterruptedException {
        bot.initC(this);

//
//        initCam(telemetry, this);
//        camOpen();

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();
        bot.passive(this);

//        bot.hookClose();
//
//        waitForStart();
//        camClose();
//        if(location == Pipeline.Location.LEFT){
//            if(whatsAuto == "redLeft"){
//                bot.drive(0, 3000, this, 0.5);
//                bot.turn(90, this, 3);//поворот на 90
//                bot.drive(0, 300,this,0.7);
//                bot.drive(2000, 0,this,0.7);
//                bot.drive(0, -5000,this,0.7);
//                bot.drive(-2000, -2000,this,0.7);
//                bot.turn(180, this, 5);//поворот на 180
//                //находим нужную зону на заднике
//                if(tagProcessor.getDetections().size() > 0){
//                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
//
//                    telemetry.addData("x", tag.ftcPose.x);
//                    telemetry.addData("y", tag.ftcPose.y);
//                    telemetry.addData("z", tag.ftcPose.z);
//                    telemetry.addData("roll", tag.ftcPose.roll);
//                    telemetry.addData("pitch", tag.ftcPose.pitch);
//                    telemetry.addData("yaw", tag.ftcPose.yaw);
//
//                    telemetry.update();
//                }
//            }
//            bot.Telescope(1000);
//            bot.sleep(500);
//            bot.upDown.setPosition(0.15);
//            bot.hookOpen();
//
//        }
//        else if(location == Pipeline.Location.CENTER){
//            bot.drive_by_time(this, 0.5,FORWARD);
//        }
//        else{
//            bot.drive_by_time(this, 0.5,RIGHT);
//        }

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

}

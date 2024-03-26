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
import org.firstinspires.ftc.teamcode.Camera.Childrens.PipeLines.PipelineReDRight;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Disabled
@Autonomous
public class Webcam extends LinearOpMode implements Direction {
    AutoMethods bot = new AutoMethods();

    protected OpenCvCamera webcam;
    protected ElapsedTime runtime = new ElapsedTime();
   public Pipeline pipe;
    public OpMode op;
    double runningtime  = 0;
    public String whatsAuto;

//    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
//            .setDrawAxes(true)
//            .setDrawCubeProjection(true)
//            .setDrawTagID(true)
//            .setDrawTagOutline(true)
//            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//            .build();
//
//    VisionPortal visionPortal = new VisionPortal.Builder()
//            .addProcessor(tagProcessor)
//            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//            .setCameraResolution(new Size(640, 480))
//            .build();


    @Override
    public void runOpMode() throws InterruptedException {
        bot.initC(this);
        initCam(telemetry, this);
        bot.initIMU(this);
        camOpen();
        bot.hook_back.setPosition(0.5);
        bot.hook_front.setPosition(0.0);
        bot.upDown.setPosition(0.4);
        runtime.reset();
        waitForStart();
        camClose();
        if(pipe.location == Pipeline.Location.LEFT){
            if(whatsAuto.equals("redLeft")){
                bot.drive(this, 0,1200, 2.5);
                bot.turn(this, -90, 1.8);
                bot.initIMU(this);
                bot.drive(this, 0,-100, 2.5);
                bot.telescope(this, 500, 2.0);
                bot.upDown.setPosition(0.87);
                sleep(500);
                bot.telescope(this, -300, 2.0);
                sleep(500);
                bot.hook_back.setPosition(0.0);
                sleep(500);
                bot.telescope(this, 150, 2.0);
                bot.upDown.setPosition(0.6);
                bot.initIMU(this);
                bot.drive(this, 2000,0, 1.8);
                bot.turn(this, 180, 1.8);
                bot.initIMU(this);
                bot.turn(this,90, 1.8);
                bot.initIMU(this);
                bot.drive(this, 0,3000, 1.8);
                bot.drive(this, 1000,0, 1.8);
                bot.upDown.setPosition(1);
                sleep(500);
                bot.hook_front.setPosition(0.3);
                sleep(500);

            } else if (whatsAuto.equals("redRight")) {
                bot.drive(this, 0,1200, 2.5);
                bot.turn(this, -90, 1.8);
                bot.initIMU(this);
                bot.drive(this, 0,-100, 2.5);
                bot.telescope(this, 500, 2.0);
                bot.upDown.setPosition(0.87);
                sleep(500);
                bot.telescope(this, -300, 2.0);
               sleep(500);
                bot.hook_back.setPosition(0.0);
                sleep(500);
                bot.telescope(this, 900, 2.0);
                bot.upDown.setPosition(0.6);
                bot.initIMU(this);
                bot.drive(this, 0,-1700, 2.5);
                bot.turn(this, 160, 1.8);
                bot.initIMU(this);
                bot.upDown.setPosition(1);;
                sleep(500);
                bot.hook_front.setPosition(0.3);
                sleep(500);

            }
        }
        else if(pipe.location == Pipeline.Location.CENTER){
            if(whatsAuto.equals("redLeft")){
                bot.drive(this, 0,1300, 1.8);
                bot.drive(this, 1500,0, 1.2);
                bot.telescope(this, 500, 2.0);
                sleep(700);
                bot.upDown.setPosition(0.0);
                sleep(700);
                bot.telescope(this, -300, 2.0);
                sleep(700);
                bot.hook_back.setPosition(0.0);
                sleep(700);
                bot.telescope(this, 300, 2.0);
                sleep(500);
                bot.upDown.setPosition(0.4);
                sleep(500);
                bot.telescope(this, -500, 2.0);
                bot.initIMU(this);
                bot.drive(this, 0,-200, 1.2);
                bot.drive(this, 500,0, 1.2);
                bot.turn(this, 90, 1.8);
                bot.initIMU(this);
                bot.drive(this, 0,-1200, 2.5);
                bot.drive(this, -6000,0, 2.5);
                bot.drive(this, 0,5000, 2.5);
                sleep(500);
                bot.telescope(this, 600, 2.0);
                sleep(500);
                bot.upDown.setPosition(0.0);
                bot.drive(this, 2000,0, 2.5);
                bot.hook_front.setPosition(0.3);
                sleep(500);
            }
            else if (whatsAuto.equals("redRight")) {
                bot.drive(this, 0,1200, 2.5);
                bot.turn(this, 45, 1.8);
                bot.initIMU(this);
                bot.telescope(this, 500, 2.0);
                sleep(500);
                bot.upDown.setPosition(0.8);
                sleep(500);
                bot.telescope(this, -300, 2.0);
                sleep(500);
                bot.hook_back.setPosition(0.0);
                sleep(500);
                bot.telescope(this, 600, 2.0);
                sleep(500);
                bot.upDown.setPosition(0.6);
                bot.initIMU(this);
                bot.turn(this, 68, 1.8);
                bot.initIMU(this);
                bot.upDown.setPosition(1);
                sleep(500);
                bot.drive(this, 0,1800, 2.5);
                bot.hook_front.setPosition(0.3);
                sleep(500);
            }
        }
        else{
            if(whatsAuto.equals("redLeft")){

            }
            else if (whatsAuto.equals("redRight")) {
                bot.drive(this, 0,1200, 2.5);
                bot.turn(this, 90, 1.8);
                bot.initIMU(this);
                bot.telescope(this, 500, 2.0);
                sleep(500);
                bot.upDown.setPosition(0.8);
                sleep(500);
                bot.telescope(this, -300, 2.0);
                sleep(500);
                bot.hook_back.setPosition(0.0);
                sleep(500);
                bot.telescope(this, 600, 2.0);
                sleep(500);
                bot.upDown.setPosition(0.6);
                bot.drive(this, 2000,0, 2.5);
                bot.drive(this, 0,1500, 2.5);
                bot.drive(this, -1000,0, 2.5);;
                bot.upDown.setPosition(1);
                sleep(500);
                bot.drive(this, 0,500, 2.5);
                bot.hook_front.setPosition(0.3);
                sleep(500);
            }
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

}

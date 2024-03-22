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
        runtime.reset();
        waitForStart();
        camClose();
        if(pipe.location == Pipeline.Location.LEFT){
            if(whatsAuto.equals("redLeft")){
//                bot.drive(this, 0, 3500, 3);
                bot.turn(this, -90, 30);
//                bot.initIMU(this);
//                bot.drive(this, 500, 0, 3);
//                bot.turn(this, 90, 2);
//                bot.initIMU(this);
//                bot.drive(this, 0, 2700, 3);
//                bot.turn(this, 90, 2.3);
//                bot.initIMU(this);
//                bot.drive(this, 0, 9500, 3.5);
//                bot.drive(this, 2500, 0, 3.5);

            }
        }
        else if(pipe.location == Pipeline.Location.CENTER){
            if(whatsAuto.equals("redLeft")){

            }
        }
        else{
            if(whatsAuto.equals("redLeft")){

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

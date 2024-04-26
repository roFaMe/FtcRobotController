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
            if (pipe.location == Pipeline.Location.LEFT) {
                if (whatsAuto.equals("blueLeft")) {
                    bot.drive(this, 0, 1450, 1.5);
                    bot.drive(this, 1000, 1450, 1.5);
                    bot.turn(this, -92, 1, 1);
                    bot.telescope(this, 500, 0.5);
                    sleep(500);
                    bot.upDown.setPosition(0.1);
                    sleep(500);
                    bot.telescope(this, -300, 0.5);
                    sleep(500);
                    bot.hook_back.setPosition(0.0);
                    sleep(500);
                    bot.telescope(this, 300, 0.5);
                    sleep(500);
                    bot.upDown.setPosition(0.0);
                    sleep(500);
                    bot.drive(this, -3000, 0, 1.5);
                    bot.drive(this, 0, 2750, 1.5);
                    bot.drive(this, 1000, 0, 1.5);
                    bot.hook_front.setPosition(0.3);
                    sleep(1000);
                    bot.telescope(this, 100, 0.5);
                    bot.upDown.setPosition(0.5);
                    bot.drive(this, -5000, 0, 2);
                }
                if (whatsAuto.equals("redRight")) {
                    bot.drive(this, 0, 1450, 1.5);
                    bot.turn(this, -96, 1, 1);
                    bot.telescope(this, 50, 0.5);
                    bot.drive(this, 0, -100, 1.5);
                    bot.telescope(this, 500, 2);
                    sleep(500);
                    bot.upDown.setPosition(0.05);
                    sleep(500);
                    bot.telescope(this, -420, 0.5);
                    sleep(500);
                    bot.hook_back.setPosition(0.05);
                    sleep(500);
                    bot.telescope(this, 300, 0.5);
                    sleep(500);
                    bot.upDown.setPosition(0.5);
                    sleep(500);
                    bot.drive(this, 0, -1525, 1.5);
                    bot.turn(this, 90, 1, 1);
                    bot.turn(this, 90, 1, 1);
                    sleep(500);
                    bot.upDown.setPosition(0.0);
                    sleep(500);
                    bot.drive(this, 0, 450, 1.5);
                    bot.drive(this, -1000, 0, 2.5);
                    sleep(500);
                    bot.hook_front.setPosition(0.3);
                    sleep(1000);
                    bot.upDown.setPosition(0.5);
                    bot.drive(this, 14000, 0, 2);
                    bot.drive(this, 0, 300, 2);
                }
                if(whatsAuto.equals("redLeft")) {
                    bot.drive(this, 0, 2500, 1.5);
                    bot.turn(this, 90, 1, 1);
                    bot.drive(this, 0, 4000, 2.5);
                    bot.drive(this, 1000, 0, 1.5);
                }
            }
            if (pipe.location == Pipeline.Location.CENTER) {
                if (whatsAuto.equals("blueLeft")) {
                    bot.drive(this, 0, 1375, 1.5);
                    bot.drive(this, 2200, 0, 1.5);
                    bot.telescope(this, 500, 0.5);
                    sleep(500);
                    bot.upDown.setPosition(0.05);
                    sleep(500);
                    bot.telescope(this, -300, 0.5);
                    sleep(500);
                    bot.hook_back.setPosition(0.0);
                    sleep(500);
                    bot.telescope(this, 300, 0.5);
                    sleep(500);
                    bot.upDown.setPosition(0.5);
                    sleep(500);
                    bot.turn(this, -96, 1, 1);
                    sleep(500);
                    bot.upDown.setPosition(0.0);
                    sleep(500);
                    bot.drive(this, 0, 2450, 1.5);
                    sleep(500);
                    bot.hook_front.setPosition(0.3);
                    sleep(1000);
                    bot.telescope(this, 100, 0.5);
                    bot.upDown.setPosition(0.5);
                    bot.drive(this, -16000, 0, 2);
                }
                if (whatsAuto.equals("redRight")) {
                    bot.drive(this, 0, 1350, 1.5);
                    bot.drive(this, -1900, 0, 1.5);
                    bot.telescope(this, 500, 2.0);
                    sleep(500);
                    bot.upDown.setPosition(0.0);
                    sleep(500);
                    bot.telescope(this, -420, 0.5);
                    sleep(500);
                    bot.hook_back.setPosition(0.0);
                    sleep(500);
                    bot.telescope(this, 300, 0.5);
                    sleep(500);
                    bot.upDown.setPosition(0.5);
                    sleep(500);
                    bot.turn(this, 95, 1, 1);
                    sleep(500);
                    bot.upDown.setPosition(0.0);
                    sleep(500);
                    bot.drive(this, 0, 3300, 1.5);
                    bot.drive(this, 1000, 0, 1.5);
                    bot.telescope(this, 100, 0.5);
                    sleep(1000);
                    bot.hook_front.setPosition(0.3);
                    sleep(1000);
                    bot.upDown.setPosition(0.5);
                    bot.drive(this, 12000, 0, 2);
                }
                if(whatsAuto.equals("redLeft")) {
                    bot.drive(this, 0, 1300, 1.5);
                    bot.drive(this, -5000, 0, 1.5);
                    bot.drive(this, 0, 1200, 1.5);
                    bot.turn(this, -90, 1,1);
                    bot.drive(this, 0, 4500, 2.5);
                    bot.drive(this, 1000, 0, 1.5);
                }
            }
            if (pipe.location == Pipeline.Location.RIGHT) {
                if (whatsAuto.equals("blueLeft")) {
                    bot.drive(this, 0, 1375, 1.5);
                    bot.turn(this, 95, 1, 1);
                    bot.telescope(this, 500, 2);
                    sleep(500);
                    bot.upDown.setPosition(0.05);
                    sleep(500);
                    bot.telescope(this, -300, 0.5);
                    sleep(500);
                    bot.hook_back.setPosition(0.0);
                    sleep(500);
                    bot.telescope(this, 300, 0.5);
                    sleep(500);
                    bot.upDown.setPosition(0.5);
                    sleep(500);
                    bot.drive(this, 0, -1275, 1.5);
                    bot.turn(this, -90, 1, 1);
                    bot.turn(this, -90, 1, 1);
                    sleep(500);
                    bot.upDown.setPosition(0.0);
                    sleep(500);
                    bot.drive(this, 0, 425, 1.5);
                    bot.drive(this, 4000, 0, 2.5);
                    bot.telescope(this, 200, 0.5);
                    sleep(500);
                    bot.telescope(this, 100, 0.5);
                    sleep(500);
                    bot.hook_front.setPosition(0.3);
                    sleep(1000);
                    bot.telescope(this, 300, 0.5);
                    bot.upDown.setPosition(0.5);
                    bot.drive(this, -12000, 0, 2);
                }
                if (whatsAuto.equals("redRight")) {
                    bot.drive(this, 0, 1350, 1.5);
                    bot.turn(this, 96, 0.8, 1);
                    bot.telescope(this, 500, 2.0);
                    sleep(500);
                    bot.upDown.setPosition(0.05);
                    sleep(500);
                    bot.telescope(this, -420, 0.5);
                    sleep(500);
                    bot.hook_back.setPosition(0.0);
                    sleep(500);
                    bot.telescope(this, 300, 0.5);
                    sleep(500);
                    bot.upDown.setPosition(0.0);
                    sleep(500);
                    bot.drive(this, 6000, 0, 2.5);
                    bot.drive(this, 0, 2800, 1.5);
                    bot.telescope(this, 100, 0.5);
                    sleep(500);
                    bot.hook_front.setPosition(0.3);
                    sleep(1000);
                    bot.upDown.setPosition(0.5);
                    bot.drive(this, 8000, 0, 2);
                }
                if(whatsAuto.equals("redLeft")) {
                    bot.drive(this, 0, 2500, 1.5);
                    bot.turn(this, 90, 1, 1);
                    bot.drive(this, 0, 4000, 2.5);
                    bot.drive(this, 1000, 0, 1.5);
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

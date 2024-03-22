package org.firstinspires.ftc.teamcode.Camera.Childrens;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.Childrens.PipeLines.PipelineBluELeft;
import org.firstinspires.ftc.teamcode.Camera.Parents.Webcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
@Autonomous(name = "CamBlueLeft", group = "Auto")
public class WebcamBlueLeft extends Webcam {

    @Override
    public void initCam(Telemetry telemetry, OpMode op){
        this.op = op;
        whatsAuto = "blueLeft";
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new PipelineBluELeft(webcam, telemetry));
    }
}

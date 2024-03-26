package org.firstinspires.ftc.teamcode.Camera.Childrens.PipeLines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.Parents.Pipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

public class PipelineBluELeft extends Pipeline {

    public PipelineBluELeft(OpenCvCamera webcam, Telemetry telemetry){
        super(webcam, telemetry);
            this.webcam = webcam;
            this.telemetry = telemetry;
        }

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);//из RGB в YCrCb
            telemetry.addLine("Pipeline running");

            Rect midleRect = new Rect(166,180, 100, 90);
            Rect rightRect = new Rect(401,221, 100, 100);

            Core.extractChannel(YCbCr, YCbCr, 1);//оставляем только СИНИЙ цвет по политре YCbCr

            Imgproc.threshold(YCbCr, YCbCr, 120, 255, Imgproc.THRESH_BINARY_INV);

            midleCrop = YCbCr.submat(midleRect);
            rightCrop = YCbCr.submat(rightRect);


            double valuemiddle = Core.sumElems(midleCrop).val[0] / midleRect.area() / 255;
            double valueright = Core.sumElems(rightCrop).val[0] / rightRect.area() / 255;

            midleCrop.release();
            rightCrop.release();


            //Процент нужного цвета в рамке
            telemetry.addData("Blue percentage in left", Math.round(valueright * 100) + "%");
            telemetry.addData("Blue percentage in middle", Math.round(valuemiddle * 100) + "%");

            Imgproc.rectangle(YCbCr, rightRect, rectColor, 2);
            Imgproc.rectangle(YCbCr, midleRect, rectColor, 2);

            if(Math.round(valueright * 100) > 10 && Math.round(valueright * 100)> Math.round(valuemiddle * 100)){
                location = Location.RIGHT;
            }else if(Math.round(valuemiddle * 100) > 10 && Math.round(valuemiddle * 100) > Math.round(valueright * 100) ){
                location = Location.CENTER;
            }else {
                location = Location.LEFT;
            }
            telemetry.addData("Локация", location);
            telemetry.update();

            return (YCbCr);
        }

}

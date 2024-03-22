package org.firstinspires.ftc.teamcode.Camera.Childrens.PipeLines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.Parents.Pipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

public class PipeLineReDLeft extends Pipeline {
       public PipeLineReDLeft(OpenCvCamera webcam, Telemetry telemetry){
                super(webcam, telemetry);
                this.webcam = webcam;
                this.telemetry = telemetry;
        }

        @Override
        public Mat processFrame(Mat input) {

                Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);//из RGB в YCrCb
                telemetry.addLine("Pipeline running");

                Rect leftRect = new Rect(0,150, 60, 100);
                Rect midleRect = new Rect(180,130, 80, 100);
                Rect rightRect = new Rect(410,150, 80, 100);

                
                Core.extractChannel(YCbCr, YCbCr, 2);//оставляем только КРАСНЫЙ цвет по политре YCbCr

                Imgproc.threshold(YCbCr, YCbCr, 120, 255, Imgproc.THRESH_BINARY_INV);

                leftCrop = YCbCr.submat(leftRect);
                midleCrop = YCbCr.submat(midleRect);
                rightCrop = YCbCr.submat(rightRect);

                double valueleft = Core.sumElems(leftCrop).val[0] / leftRect.area() / 255;
                double valuemiddle = Core.sumElems(midleCrop).val[0] / midleRect.area() / 255;
                double valueright = Core.sumElems(rightCrop).val[0] / rightRect.area() / 255;

                leftCrop.release();
                midleCrop.release();
                rightCrop.release();


                //Процент нужного цвета в рамке
                telemetry.addData("RED percentage in left", Math.round(valueleft * 100) + "%");
                telemetry.addData("RED percentage in middle", Math.round(valuemiddle * 100) + "%");
                telemetry.addData("RED percentage in right", Math.round(valueright * 100) + "%");

                Imgproc.rectangle(YCbCr, leftRect, rectColor, 2);
                Imgproc.rectangle(YCbCr, midleRect, rectColor, 2);
                Imgproc.rectangle(YCbCr, rightRect, rectColor, 2);

                if(Math.round(valueright * 100) > 15 && Math.round(valueright * 100)> Math.round(valuemiddle * 100) && Math.round(valueright * 100)> Math.round(valueleft * 100)){
                    location = Location.RIGHT;
                }else if(Math.round(valuemiddle * 100) > 15 && Math.round(valuemiddle * 100) > Math.round(valueleft * 100) && Math.round(valuemiddle * 100) > Math.round(valueright * 100) ){
                    location = Location.CENTER;
                }else {
                    location = Location.LEFT;
                }
                telemetry.addData("Локация", location);
                telemetry.update();
                return (YCbCr);
        }
}
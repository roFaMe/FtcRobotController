package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Camera.Childrens.WebcamRedLeft;

@Autonomous(name="RedLeft", group="Auto")

public class AutoRedLeft extends WebcamRedLeft {
    AutoMethods bot = new AutoMethods();
    @Override
    public void runOpMode() throws InterruptedException {

        bot.initIMU(this);
        bot.initC(this);

        waitForStart();

        bot.drive(this, 0.5);
    }
}

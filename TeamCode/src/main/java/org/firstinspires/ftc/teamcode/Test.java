package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test", group="Auto")
public class Test extends AutoMethods  implements Direction {
    AutoMethods bot = new AutoMethods();

    @Override
    public void runOpMode () {
        bot.initC(this);

        waitForStart();

        bot.drive(this, 0 ,150,30);


    }

}
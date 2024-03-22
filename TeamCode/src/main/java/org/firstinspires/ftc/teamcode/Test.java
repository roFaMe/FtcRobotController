package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test", group="Auto")
public class Test extends AutoMethods  implements Direction{
    AutoMethods bot = new AutoMethods();

    @Override
    public void runOpMode (){
        bot.initC(this);
        bot.initIMU(this);
        waitForStart();

//        bot.drive(this, 6000, 0, 50);
//        sleep(600);
        bot.turn(this, 90.0, 3);
        bot.initIMU(this);
//        sleep(600);
//        bot.drive(this, 0, 5500, 50);

    }

}
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

        bot.drive(this, 0, 2500, 30);
//        bot.drive(this, 2500, 0, 2);
//        sleep(600);
//        bot.drive(this, 4500, 0, 50);
//        sleep(600);
//        bot.turn(this, -90.0, 2.5);
//        bot.initIMU(this);
    }

}
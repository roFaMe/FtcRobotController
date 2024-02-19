package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="ParkovkaRedLeft", group="Auto")
public class ParkovkaRedLeft extends AutoMethods  implements Direction {
    AutoMethods bot = new AutoMethods();

    @Override
    public void runOpMode (){
        bot.initC(this);

        waitForStart();

        bot.drive_by_time(this, 0.75, FORWARD);
        sleep(1000);
        bot.drive_by_time(this, 0.55, RIGHT);
        bot.drive_by_time(this, 1.5, FORWARD);
    }

}


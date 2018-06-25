package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by nehapant on 6/24/18.
 */

@Autonomous(name="RedRelic85Autonomous", group="Team5214")

public class RedRelic85Autonomous extends RedAutonomous{

    @Override
    protected void getToLeftDumpPosition(){
        straightWithEncoder(.55, -31);
        leftDump.setPosition(.61);
        turnRightDegrees(53, parameters);
    }


    @Override
    protected void getToCenterDumpPosition(){
        straightWithEncoder(.5, -24);
        leftDump.setPosition(.61);
        turnRightDegrees(55, parameters);
    }

    @Override
    protected void getToRightDumpPosition(){
        straightWithEncoder(.5, -35);
        leftDump.setPosition(.61);
        turnRightDegrees(105, parameters);
    }

    @Override
    protected void getToDefaultDumpPosition(){
        getToCenterDumpPosition();
    }


}

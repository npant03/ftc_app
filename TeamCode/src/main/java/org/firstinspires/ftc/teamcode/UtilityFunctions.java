package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by nehapant on 6/20/18.
 */

public class UtilityFunctions{

    public static void  motorWithEncoder(DcMotor motorName, double power, int inches) {
        int ticks = (int) (inches * 1120 / (4 * 3.14159)); //converts inches to ticks
//      telemetry.update()
        //modifies moveto position based on starting ticks position, keeps running tally
        int position2move2 = motorName.getCurrentPosition() + ticks;
        motorName.setTargetPosition(position2move2);
        motorName.setPower(power);

    }
    public static void sleep(int i) {
        //initial time takes the current hardware time in milliseconds
        long initial_time = System.currentTimeMillis();
        //inside the while loop cpu will stop working when the input time is more than the time passed in this loop
        //cpu will be back working when the loop reaches the target time
        while (System.currentTimeMillis() - initial_time < i) {

        }
    }

}

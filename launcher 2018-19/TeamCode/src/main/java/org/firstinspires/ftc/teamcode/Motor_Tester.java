package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Lift test", group="Iterative Opmode")
//@Disabled

public class Motor_Tester extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FT_Robot robot =  new FT_Robot();
    double val = 0;
     /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        robot.initTeleOp(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    if (gamepad1.dpad_up && val <= 1) {
        val += 0.005;
    } else if (gamepad1.dpad_down && val >= -1) {
        val -= 0.005;
    }
    if (gamepad1.a){
        robot.cageLiftL.setPower(val * -1);
        robot.cageLiftR.setPower(val);
    }else{
        robot.cageLiftR.setPower(0);
        robot.cageLiftL.setPower(0);
        val *= 0;
    }


    // Show the elapsed game time and wheel power.
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("Motors", "SPEED (%.3f)", val);
        telemetry.addData("Motors", "yeetR (%.3f), yeetL (%.3f)", robot.cageLiftR.getPower(), robot.cageLiftL.getPower());
    telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

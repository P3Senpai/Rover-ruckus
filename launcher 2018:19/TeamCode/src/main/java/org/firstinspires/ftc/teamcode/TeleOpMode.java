package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Main", group="Iterative Opmode")
//@Disabled

public class TeleOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FT_Robot robot =  new FT_Robot();

    private String elementColour;
    private int whiteColor;
    private int yellowColor;

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


    // Set up driving so that robot can be controlled with 1 joystick (gp1, left)
    double drive  = -gamepad1.left_stick_y;
    double turn   =  gamepad1.left_stick_x;
    double leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
    double rightPower   = Range.clip(drive + turn, -1.0, 1.0) ;

        //Left Drive Power
    robot.leftDriveF.setPower(leftPower);    // TODO: Check if the back left motor isn't slower than the others
    robot.leftDriveB.setPower(leftPower);
        // Right Drive power
    robot.rightDriveF.setPower(rightPower);
    robot.rightDriveB.setPower(rightPower);

    // Manual lift code below
        double leftLiftPow  = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);
        double rightLiftPow = Range.clip(gamepad2.right_stick_y, -1.0, 1.0);
        robot.cageLiftL.setPower(leftLiftPow * 0.5);
        robot.cageLiftR.setPower(rightLiftPow * 0.5);
        //TODO: test how automatic lifting and manual lifting work in unison


    // Automized lifting
        // make reset encoders before setting pos                    // THIS SHOULD WORK IF CODE RUNS PROCEDURALLY
        // if statement that only sets pos if on extremes and NOT when busy
        // Second button tracker (if pressed) that only runs motors to position

    // test of cage lift automatic controls
    if(gamepad1.y && !robot.lastLiftB && !robot.liftIsBusy()){
        robot.lastLiftB = true;
        // if auto motor doesn't work then add setMode (run-to-pos) here
        int currPos = robot.cageLiftRAuto.getCurrentPosition(); // TODO: find acutal positions
        if(currPos == 100){
            robot.cageLiftRAuto.setTargetPosition(0);
            robot.cageLiftLAuto.setTargetPosition(0);
            robot.cageLiftRAuto.setPower(0.2);
            robot.cageLiftLAuto.setPower(0.2);
        } else if (currPos == 0){
            robot.cageLiftRAuto.setTargetPosition(100);
            robot.cageLiftLAuto.setTargetPosition(100);
            robot.cageLiftRAuto.setPower(0.2);
            robot.cageLiftLAuto.setPower(0.2);
        }
    } else if (robot.lastLiftB) {
        robot.lastLiftB = false;
    }


        // cage intake code below
    if (gamepad2.a)
        robot.cageIntake.setPower(robot.INTAKE_SPEED);
    else if (gamepad2.b)
        robot.cageIntake.setPower(robot.INTAKE_SPEED_OUT);
    else
        robot.cageIntake.setPower(0.0);


        // Robot lift controls
    if(gamepad1.dpad_up)
        robot.roboLift.setPower(1.0);
    else if(gamepad1.dpad_down)
        robot.roboLift.setPower(-1.0);
    else
        robot.roboLift.setPower(0);


        // Marker Drop only test for autonomous
    if (gamepad1.a)
        robot.markerDrop.setPosition(0.2);
    else if (gamepad1.b)
        robot.markerDrop.setPosition(0.8);
    else
        robot.markerDrop.setPosition(0.5);

        // Automized crater lift
//        robot.overCrater();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*  TODO: find value ranges
     * gets rgb colour values and returns: White, Yellow, Black
     *  TODO: should i incorporate alpha() values ????????????????????
     */
    private String rgbValueCalc(int rawBlue, int rawGreen, int rawRed){
        double scaleFactor = 255;
        int cleanBlue = (int) (rawBlue * scaleFactor);
        int cleanGreen = (int) (rawGreen * scaleFactor);
        int cleanRed = (int) (rawRed * scaleFactor);

//          TODO: find value ranges
//        if (/*yellow*/){
//            return "Yellow";
//            } //else if (){
//            return "White";
//       }
            return "Nothing";
    }
}

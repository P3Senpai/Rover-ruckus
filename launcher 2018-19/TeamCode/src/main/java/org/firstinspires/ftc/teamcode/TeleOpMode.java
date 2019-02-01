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

    // region driving
    // Set up driving so that robot can be controlled with 1 joystick (gp1, left)
    double drive  = -gamepad1.left_stick_y;
    double turn   =  gamepad1.left_stick_x;
    double leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
    double rightPower   = Range.clip(drive + turn, -1.0, 1.0) ;

        //Left Drive Power
    robot.leftDriveF.setPower(leftPower * -1);
    robot.leftDriveB.setPower(leftPower);
        // Right Drive power
    robot.rightDriveF.setPower(rightPower * -1);
    robot.rightDriveB.setPower(rightPower);
    // endregion
    // region element lift
    // Manual lift code below
        double liftPower  = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);
        robot.cageLiftL.setPower(liftPower * robot.LIFT_POWER_CAP);
        robot.cageLiftR.setPower(liftPower * robot.LIFT_POWER_CAP );
        manualToAuto(liftPower, gamepad2.left_stick_button);


    // Automatized lifting
    if(gamepad2.a && !robot.lastLiftB && !robot.liftIsBusy()){
        // local values
        int currPosL = robot.cageLiftL.getCurrentPosition();
        int currPosR = robot.cageLiftR.getCurrentPosition();
        // Change toggle state
        robot.lastLiftB = true;
        // Set up automatized values
        robot.liftSetup('a');
        // Toggle condition path
        if(currPosL == 900 && currPosR == 900){
            robot.cageLiftR.setTargetPosition(0);
            robot.cageLiftL.setTargetPosition(0);
            robot.cageLiftR.setPower(0.15);
            robot.cageLiftL.setPower(0.15);
        } else if (currPosL == 0 && currPosR == 0){
            robot.cageLiftR.setTargetPosition(900);
            robot.cageLiftL.setTargetPosition(900);
            robot.cageLiftR.setPower(0.45);
            robot.cageLiftL.setPower(0.45);
        }
    } else if (!robot.liftIsBusy()) {
        robot.liftSetup('m');
    }else if (robot.lastLiftB){
        robot.lastLiftB = false;
    }

        // Automatized over crater lift
        // imu values
        double heading = robot.imu.getAngularOrientation().firstAngle;
        double roll  = robot.imu.getAngularOrientation().secondAngle;
        double pitch = robot.imu.getAngularOrientation().thirdAngle;
//        robot.overCrater();

    // endregion

    // region cage intake
        // cage intake code below  // TODO: remap buttons to driver preferences
    if (gamepad2.a)
        robot.cageIntake.setPower(robot.INTAKE_SPEED);
    else if (gamepad2.b)
        robot.cageIntake.setPower(robot.INTAKE_SPEED_OUT);
    else
        robot.cageIntake.setPower(0.0);
    // endregion

        // Robot lift controls  TODO: Make lifting limits to stop the rail from breaking
        int roboLiftPos = robot.roboLift.getCurrentPosition();
    if(gamepad2.dpad_up ) { // && roboLiftPos <= 0
        robot.roboLift.setPower(1.0);
    }
    else if(gamepad2.dpad_down ){ // && roboLiftPos >= -15853
        robot.roboLift.setPower(-1.0);
    }
    else
        robot.roboLift.setPower(0);

    //TODO: move to autonomous op mode
    // region marker drop
        // Marker Drop only test for autonomous
        double pos = robot.markerDrop.getPosition();
    if (gamepad1.a)
        robot.markerDrop.setPosition(pos+ 0.05);
    else if (gamepad1.b)
        robot.markerDrop.setPosition(pos- 0.05);

    // endregion

        // region color move
        double colorS = robot.colorMove.getPosition();
        if (gamepad1.x)
//            robot.colorMove.setPosition(colorS + 0.05);
            robot.colorMove.setPosition(0.0);
        else if (gamepad1.y)
//            robot.colorMove.setPosition(colorS - 0.05);
            robot.colorMove.setPosition(0.5);
        // endregion

        // region color move
        double releaseS = robot.liftRelease.getPosition();
        if (gamepad2.x)
//            robot.liftRelease.setPosition(releaseS + 0.05);
            robot.liftRelease.setPosition(0.2);
        else if (gamepad2.y)
//            robot.liftRelease.setPosition(releaseS - 0.05);
            robot.liftRelease.setPosition(0.8);
        // endregion

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Motor Pos", "left %d, right %d", robot.cageLiftL.getCurrentPosition(), robot.cageLiftR.getCurrentPosition());
        telemetry.addData("ROBOT LIFT POS:", " %d", roboLiftPos);
        telemetry.addData("Servo", "(%.2f)", robot.liftRelease.getPosition());
        telemetry.addData("IMU: ", "heading(z) (%.2f), roll(x) (%.2f), pitch(y) (%.2f)", heading, roll, pitch);
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
        if ((rawBlue < 100) &&
                (rawGreen > 200) &&
                ( rawRed> 200)
              ){
            return "Yellow";
            } else if (rawBlue == rawRed &&
                        rawBlue == rawGreen &&
                        rawGreen == rawRed &&
                        rawBlue > 200){
            return "White";
       }
            return "Nothing";
    }

    // TODO: Change manual nums to variables

    private void manualToAuto(double direction, boolean button){
        // Set automatic lift set run mode
        robot.liftSetup('a');
        // Deciding where to clip lifts
        if ((direction >= (0.3 * robot.LIFT_POWER_CAP)) && button){
            robot.cageLiftL.setTargetPosition(900);
            robot.cageLiftR.setTargetPosition(900);
            robot.cageLiftL.setPower(0.45);
            robot.cageLiftR.setPower(0.45);
        } else if ((direction <= (-0.3 * robot.LIFT_POWER_CAP)) && button){
            robot.cageLiftL.setTargetPosition(0);
            robot.cageLiftR.setTargetPosition(0);
            robot.cageLiftL.setPower(0.15);
            robot.cageLiftR.setPower(0.15);
        }
    }
}

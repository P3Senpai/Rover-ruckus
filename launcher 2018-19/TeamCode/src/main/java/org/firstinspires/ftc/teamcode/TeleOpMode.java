package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    robot.leftDrive.setPower(leftPower);
        // Right Drive power
    robot.rightDrive.setPower(rightPower);
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

    // region arm pivot
        double pivotPower = Range.clip(gamepad2.right_stick_y, -1.0, 1.0);
        pivotPower *= 0.6; // scale of pivot power
        robot.pivotArm.setPower(pivotPower);
        robot.extendingBig.setPower(pivotPower/robot.largePulleyRotation);
        robot.extendingSmall.setPower(pivotPower/robot.largePulleyRotation);
        robot.extendingPull.setPower(pivotPower/robot.tighteningPullyRotation);
    // endregion

    // region Robot lift controls  TODO: Make lifting limits to stop the rail from breaking
        int roboLiftPos = robot.roboLift.getCurrentPosition();
    if(gamepad2.dpad_up ) { // && roboLiftPos <= 0
        robot.roboLift.setPower(1.0);
    }
    else if(gamepad2.dpad_down ){ // && roboLiftPos >= -15853
        robot.roboLift.setPower(-1.0);
    }
    else
        robot.roboLift.setPower(0);
    // endregion
    //TODO: move to autonomous op mode
    // Todo; find if this necessary
    // region marker drop
        // Marker Drop only test for autonomous
        double pos = robot.markerDrop.getPosition();
    if (gamepad1.a)
        robot.markerDrop.setPosition(pos+ 0.05);
    else if (gamepad1.b)
        robot.markerDrop.setPosition(pos- 0.05);

    // endregion

        // region lift release
        double releaseS = robot.liftRelease.getPosition();
        if (gamepad2.x)
//            robot.liftRelease.setPosition(releaseS + 0.05);
            robot.liftRelease.setPosition(0.2);
        else if (gamepad2.y)
//            robot.liftRelease.setPosition(releaseS - 0.05);
            robot.liftRelease.setPosition(0.8);
        // endregion

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}

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
    double drive  = -gamepad1.left_stick_y;  //todo check driving directions
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
            robot.pivotArm.setPower(pivotPower);


    // endregion

    // region arm extension
        double leftTrigger = Range.clip(gamepad2.left_trigger, 0.0, 1.0);
        double rightTrigger = Range.clip(gamepad2.right_trigger, 0.0, 1.0);
    // endregion

    // region Robot lift controls  TODO: Make lifting limits to stop the rail from breaking
        int roboLiftPos = robot.roboLift.getCurrentPosition();
    if(gamepad1.dpad_up ) { // && roboLiftPos <= 0
        robot.roboLift.setPower(1.0);
    }
    else if(gamepad1.dpad_down){ // && roboLiftPos >= -15853
        robot.roboLift.setPower(-1.0);
    }
    else
        robot.roboLift.setPower(0);
    // endregion

        // todo add data
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("dir", "big %s small %s", robot.extendingPulley.getDirection().toString(), robot.extendingSprocket.getDirection().toString());
        telemetry.addData("Triggers: ", "L-(%.2f) R-(%.2f)", leftTrigger, rightTrigger);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void linearMotion(int max, int min, DcMotor motor, int currPos, double power){
        // Setting up variables
        int maxRotationPerCycle = 70; // 70 in encoder value, which is equivalent to 0.125 rotations at 300rpm per 25ms
        int appendRotation = (int) (power * maxRotationPerCycle);
        double speed = Math.abs(power);

        // checks if motor is outside range between min and max
        if ((currPos < min) || (max < currPos)){       // if not within range
            if (currPos < min && power > 0) {   // if above range and moving down
                motor.setTargetPosition(currPos + appendRotation);
                motor.setPower(speed);
            } else if (currPos > max && power < 0) {  // if below range and moving up
                motor.setTargetPosition(currPos + appendRotation);
                motor.setPower(speed);
            }else {                                                 // Else stop moving
                motor.setTargetPosition(currPos);
                motor.setPower(0);
            }
        } else {
            motor.setTargetPosition(currPos + appendRotation);
            motor.setPower(speed);
        }
    }
}

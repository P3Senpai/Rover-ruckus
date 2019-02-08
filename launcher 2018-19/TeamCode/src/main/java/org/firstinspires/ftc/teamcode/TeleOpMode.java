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

    // region gustav's controls (gp1)
        // region driving
    // Set up driving so that robot can be controlled with 1 joystick (gp1, left)
    double drive  = -gamepad1.left_stick_y;  //todo check driving directions
    double turn   =  gamepad1.left_stick_x;
    double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
    double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        //Left Drive Power
    robot.leftDrive.setPower(leftPower);
        // Right Drive power
    robot.rightDrive.setPower(rightPower);
        // Extra gusti controls
    if (gamepad1.dpad_up){
        robot.leftDrive.setPower(1.0);
        robot.rightDrive.setPower(1.0);
    }else if(gamepad1.dpad_down){
        robot.leftDrive.setPower(-1.0);
        robot.rightDrive.setPower(-1.0);
    }else if(gamepad1.dpad_left){
        robot.leftDrive.setPower(-1.0);
        robot.rightDrive.setPower(1.0);
    }else if(gamepad1.dpad_right){
        robot.leftDrive.setPower(1.0);
        robot.rightDrive.setPower(-1.0);
    }else{
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
    }
    // endregion
        // region Robot lift controls
        if(gamepad1.y) { // && roboLiftPos <= 0
            robot.roboLift.setPower(1.0);
        }
        else if(gamepad1.a){ // && roboLiftPos >= -15853
            robot.roboLift.setPower(-1.0);
        }
        else
            robot.roboLift.setPower(0);
        // endregion

    // endregion

    // region Jun Sang's code (gp2)
        // region cage intake
        // cage intake code below  // TODO: remap buttons to driver preferences
    if (gamepad2.a)
        robot.cageIntake.setPower(1.0);
    else if (gamepad2.b)
        robot.cageIntake.setPower(-0.6);
    else
        robot.cageIntake.setPower(0.0);
    // endregion
        // region arm pivot
        double pivotPower = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);
        robot.pivotArm.setPower(pivotPower * 0.6);
        robot.extendingPulley.setPower(pivotPower * 0.4);
        robot.extendingSprocket.setPower(pivotPower * 0.4);
        robot.extendingPull.setPower(pivotPower * 0.3);

    // endregion
        // region arm extension
        double rightJoy = Range.clip(gamepad2.right_stick_y, -1.0, 1.0);
        robot.extendingSprocket.setPower(rightJoy);
        robot.extendingPulley.setPower(rightJoy);
        robot.extendingPull.setPower(rightJoy * 0.6);
    // endregion

    // endregion

        // testing pin release servo
        double pos = robot.liftRelease.getPosition();
        if (gamepad1.a){
            robot.liftRelease.setPosition(pos + 0.05);
        }else if(gamepad1.b){
            robot.liftRelease.setPosition(pos - 0.05);
        }

        // todo add data
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("dir", "big %s small %s", robot.extendingPulley.getDirection().toString(), robot.extendingSprocket.getDirection().toString());
        telemetry.addData("Servo pull: ", "Pos:(%.2f)", robot.liftRelease.getPosition());
        telemetry.addData("Pulley test", "Pos: %d", robot.extendingPull.getCurrentPosition());
        telemetry.addData("Heading:", "(%.2f)", robot.imu.getAngularOrientation().firstAngle);
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
                motor.setPower(1.0);
            } else if (currPos > max && power < 0) {  // if below range and moving up
                motor.setTargetPosition(currPos + appendRotation);
                motor.setPower(1.0);
            }else {                                                 // Else stop moving
                motor.setTargetPosition(currPos);
                motor.setPower(0);
            }
        } else {
            motor.setTargetPosition(currPos + appendRotation);
            motor.setPower(1.0);
        }
    }
}

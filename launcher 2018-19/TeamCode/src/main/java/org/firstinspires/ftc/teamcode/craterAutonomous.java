/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="crater Autonomous", group="Autonomous")
//@Disabled
public class craterAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    FT_Robot robot   = new FT_Robot();
    static String mineralPos; /** My code */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        TenserFlow tf = new TenserFlow(); /** My code */
        robot.initAutonomous(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
/********************************************************************************/

        // drop from lander
        // todo check if i want advance before cube
        tf.runOpMode(); /** My code */
        turnByAngle(0.4, 90,3);
        if(mineralPos.equals("center")){
            moveByEncoder(0.6, 68,68,7); // original diastance +5
            moveByEncoder(0.6, -68,-68,7);
            // todo go to depot
            // todo drop marker
            // go to crater
            turnByAngle(0.4, 180, 5);
            moveByEncoder(0.6,195,195, 10);

        }else if(mineralPos.equals("left")){
            moveByEncoder(0.6, 23,23,3); // go to x
            turnByAngle(0.4, 42.208,2); // turn to mineral
            moveByEncoder(0.6, 59,59,3); //sample mineral +5cm of original dis
            // go back to x
            moveByEncoder(0.6, -59,-59,3);
            turnByAngle(0.4, -42.208,2);
            // todo go to depot
            //todo drop marker
            // move to crater
            turnByAngle(0.4, 180, 2);
            moveByEncoder(0.6,195,195, 10);
        }
        else if(mineralPos.equals("right")){
            moveByEncoder(0.6, 23,23,3); // go to x
            turnByAngle(0.4, -42.208,2); // turn to mineral
            moveByEncoder(0.6, 59,59,3); // sample mineral +5cm of original dis
            moveByEncoder(0.6, -54,-54,3);
            turnByAngle(0.4, 42.208,2);
            //todo go to depot
            //todo drop marker
            // move to crater
            turnByAngle(0.4, 180, 2);
            moveByEncoder(0.6,195,195, 10);

        }

        telemetry.addData("Path", "Complete");
        telemetry.addData( "Mineral Position: ", mineralPos);
        telemetry.update();
    }

    // todo add move by encoder method

    private void moveByEncoder(double speed, double leftDistance, double rightDistance, double timeOut){
        ElapsedTime timer = new ElapsedTime();
        double wheelToDistance = 2 * Math.PI * 45;
        // calculate target distance
        int leftCurrentPosition = robot.leftDrive.getCurrentPosition();
        int rightCurrentPostition = robot.rightDrive.getCurrentPosition();

        int leftTarget = leftCurrentPosition + (int) (leftDistance * robot.drivingWheelToCm);
        int rightTarget = rightCurrentPostition + (int) (rightDistance* robot.drivingWheelToCm);

        // convert motor to run to pos
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set target pos
        robot.leftDrive.setTargetPosition(leftTarget);
        robot.rightDrive.setTargetPosition(rightTarget);

        // Reset and start
        timer.reset();
        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                timer.seconds() < timeOut &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
                ){
            telemetry.addData("Current driving position: ", " L:%7d R:%7d", leftCurrentPosition, rightCurrentPostition);
            telemetry.addData("Distance left till target: ", " L:%7d R:%7d", leftTarget - leftCurrentPosition, rightTarget - rightCurrentPostition);
            telemetry.addData("Time left: ", "(%.2f)", timeOut-timer.seconds());
            telemetry.update();
        }
        telemetry.addLine("Driving complete :)");
        telemetry.update();

        // Stop all motors
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);

        // Turn off run to position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    private void turnByAngle(double speed, double turnAngle, double timeOut){
        // set up of parameters
        ElapsedTime timer = new ElapsedTime();
        double currAngle = robot.imu.getAngularOrientation().thirdAngle;
        double targetAngle = Math.abs((currAngle + turnAngle));
        speed = Math.abs(speed);
        // other var
        double leftMotorDirection;
        double rightMotorDirection;

        // spin left or right
        if (turnAngle < 180){
            leftMotorDirection = -1;
            rightMotorDirection = 1;
        }else{
            leftMotorDirection = 1;
            rightMotorDirection = -1;
        }

        timer.reset();
        while(opModeIsActive() &&
               timer.seconds() <= timeOut &&
                currAngle != turnAngle){
            robot.leftDrive.setPower(speed * leftMotorDirection);
            robot.rightDrive.setPower(speed * rightMotorDirection);
            telemetry.addData("Current degrees: ", currAngle);
            telemetry.addData("Degrees left till target position: ", turnAngle - currAngle);
            telemetry.addData("Time left: ", "(%.2f)", timeOut-timer.seconds());
            telemetry.update();
        }
        telemetry.addLine("Turn Complete :)");
        telemetry.update();

        // Stop all motors
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);

        // Turn off run to position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void servoMotion(Servo servo, double endPos, double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() &&
                holdTimer.time() <= holdTime){
            servo.setPosition(endPos);
        }
    }
}

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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class FT_Robot
{
    /* Public OpMode members. */
    protected DcMotor  leftDriveF   = null;
    protected DcMotor  rightDriveF  = null;
    protected DcMotor  leftDriveB   = null;
    protected DcMotor  rightDriveB  = null;
    protected DcMotor  cageLiftL    = null;
    protected DcMotor  cageLiftR    = null;
    protected DcMotor  cageIntake   = null;
    protected DcMotor  roboLift     = null;
    protected Servo    markerDrop    = null;

    protected static final double SPEED_REDUCTION = 0.9;
    protected static final double INTAKE_IN_SPEED = 0.1;
    protected static final double INTAKE_OUT_SPEED = 0.9;

    protected SensorIMU imuSensor =null;
    protected double heading = imuSensor.angles.firstAngle;
    protected double roll = imuSensor.angles.secondAngle;
    protected double pitch = imuSensor.angles.thirdAngle;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public FT_Robot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
            // Drive train
        leftDriveF  = hwMap.get(DcMotor.class, "left_drive_f");
        rightDriveF = hwMap.get(DcMotor.class, "right_drive_f");
        leftDriveB  = hwMap.get(DcMotor.class, "left_drive_b");
        rightDriveB = hwMap.get(DcMotor.class, "right_drive_b");
            // Cage Lift
        cageLiftL   = hwMap.get(DcMotor.class, "cage_lift_l");
        cageLiftR   = hwMap.get(DcMotor.class, "cage_lift_r");
            // Robot lifting
        roboLift    = hwMap.get(DcMotor.class, "robo_lift");
            // Cage intake
        cageIntake  = hwMap.get(DcMotor.class, "cage_intake");

        leftDriveF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDriveF.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftDriveB.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDriveB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        cageLiftL.setDirection(DcMotor.Direction.FORWARD);
        cageLiftR.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        leftDriveF.setPower(0);
        rightDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveB.setPower(0);
        cageLiftL.setPower(0);
        cageLiftR.setPower(0);
        roboLift.setPower(0);
        cageIntake.setPower(0);

        // Set ground positions motor positions
//        cageLiftL.setTargetPosition(0);
//        cageLiftR.setTargetPosition(0);
        // TODO: add encoder init for the cage lift

        //Initialize all servo
        markerDrop  = hwMap.get(Servo.class, "marker_drop");
    }
 }


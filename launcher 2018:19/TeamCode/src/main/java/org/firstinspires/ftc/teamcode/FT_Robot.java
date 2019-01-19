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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FT_Robot {
    /* Dc motors  */
    // Drive train motors
    protected DcMotor leftDriveF = null;
    protected DcMotor rightDriveF = null;
    protected DcMotor leftDriveB = null;
    protected DcMotor rightDriveB = null;
    // Cage Lift and Intake motors
    protected DcMotor cageLiftL = null;
    protected DcMotor cageLiftR = null;
    protected DcMotor cageIntake = null;
    // Robot lift motor
    protected DcMotor roboLift = null;

    /* Servos */
    protected Servo markerDrop = null;

    /* Sensors */
    protected SensorIMU imuSensor = null;


    /* Preset Values */
    // Intake speeds
    protected final static double INTAKE_SPEED = 1.0;
    protected final static double INTAKE_SPEED_OUT = -0.4;

    // Lifting position         // TODO: Check all of the encoder numbers
    protected final int TOP_LIFT = 1000;
    protected final int GROUND_LIFT = 0;
    protected final int CRATER_LIFT = 200;

    //Toggle Values
    protected boolean lastLiftB = false;
    // Calibrated angle on flat surface
    protected final double FLAT_SOURCE = 85.0;   // TODO: Calibrate every time you go somewhere new

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public FT_Robot() {
    }

    /* Initialize Hardware interfaces */
    public void initTeleOp(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        // Drive train
        leftDriveF = hwMap.get(DcMotor.class, "left_drive_f");
        rightDriveF = hwMap.get(DcMotor.class, "right_drive_f");
        leftDriveB = hwMap.get(DcMotor.class, "left_drive_b");
        rightDriveB = hwMap.get(DcMotor.class, "right_drive_b");
        // Cage Lift
        cageLiftL = hwMap.get(DcMotor.class, "cage_lift_l");
        cageLiftR = hwMap.get(DcMotor.class, "cage_lift_r");
        cageLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cageLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Robot lifting
        roboLift = hwMap.get(DcMotor.class, "robo_lift");
        // Cage intake
        cageIntake = hwMap.get(DcMotor.class, "cage_intake");

        // Set drive train directions to motors
        leftDriveF.setDirection(DcMotor.Direction.FORWARD);
        rightDriveF.setDirection(DcMotor.Direction.REVERSE);
        leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        rightDriveB.setDirection(DcMotor.Direction.REVERSE);

        // Cage lift Directions
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

        // Cage lift manual controls init
        cageLiftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cageLiftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: For init set ground positions to lift motors positions

        //Initialize all servo
        markerDrop = hwMap.get(Servo.class, "marker_drop");

        //TODO: add starting pos of ALL servos
        markerDrop.setPosition(0.5);

    // imu init
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    // TODO: add ALL hwmap init to method below
    public void initAutonomous(HardwareMap ahwMap) {
    }

    /* Methods for all op modes*/

    // TODO: Correct degree variation in if statement so that it works well
//    public boolean overCrater(double currentAngle) {
//        if ((currentAngle > (FLAT_SOURCE + 4)) || (currentAngle < (FLAT_SOURCE - 4))) {
//            cageLiftL.setTargetPosition(CRATER_LIFT);
//            cageLiftR.setTargetPosition(CRATER_LIFT);
//            cageLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            cageLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            cageLiftL.setPower(0.2);
//            cageLiftR.setPower(0.2);
//            return true;
//        } else {
//            cageLiftL.setTargetPosition(GROUND_LIFT);
//            cageLiftR.setTargetPosition(GROUND_LIFT);
//            cageLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            cageLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            cageLiftL.setPower(0.2);
//            cageLiftR.setPower(0.2);
//            return false;
//        }
//    }

    protected boolean liftIsBusy() {
        if (cageLiftL.isBusy() && cageLiftR.isBusy()) {
            return true;
        } else {
            return false;
        }
    }

    // inputs: 'a' or 'm'--> m means manual and a means automatic
    // this setup method will be useful if there can't be 2 variables on the same motor
    protected void liftSetup(char mode) {
        if (mode == 'a') {
            cageLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cageLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (mode == 'm'){
            cageLiftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cageLiftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}


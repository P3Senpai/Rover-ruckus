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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FT_Robot {
    /* Dc motors  */
    // Drive train motors
    protected DcMotor leftDrive = null;
    protected DcMotor rightDrive = null;
    // Linear extension and Intake motor
    protected DcMotor pivotArm = null;
    protected DcMotor extendingBig = null;
    protected DcMotor extendingSmall = null;
    protected DcMotor extendingPull = null;
    protected DcMotor cageIntake = null;
    // Robot lift motor
    protected DcMotor roboLift = null;
    /* Servos */
    protected Servo markerDrop = null;
    protected Servo liftRelease = null;

    /* Sensors */
    BNO055IMU imu; // The IMU sensor object


    /* Preset Values */
    // Intake speeds
    protected final double INTAKE_SPEED = 1.0;
    protected final double INTAKE_SPEED_OUT = -0.6; // TODo: test if speeds apply to new design

    // TODO measure lenght in mm not cm
    // TODO find x
    // x = difference between starting and ending pos
    double smallPulleyRotation =  x / (2*Math.PI*15*2); // Diameter of pulley is 30 mm * 2:1 gear ratio
    double largePulleyRotation = x / (2*Math.PI*30); // Diameter of pulley is 60 mm
    double tighteningPullyRotation = x / (2*Math.PI* y); // TODO find out pulley size ( y = pull radius)
    // TODO find small pulley gear ratio

    // max length for linear extension
    // todo find x
    // x = difference between the max and min len of string
    int smallPulleyMax = (int) x / (2*Math.PI*15*2); // 15 = diameter of 30mm pulley, 2 = 2:1 gear ratio
    int bigPulleyMax = (int) x / (2*Math.PI*30); // 30 = diameter of 60 mm pulley
    int tighteningPulleyMax = (int) x / (2*Math.PI* y); // y = diameter pulley, add gear ratio of there is one

    // The IMU sensor object
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
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        extendingBig = hwMap.get(DcMotor.class, "extending_big");
        extendingSmall = hwMap.get(DcMotor.class, "extending_small");
        extendingPull = hwMap.get(DcMotor.class, "extending_pull");
        roboLift = hwMap.get(DcMotor.class, "robo_lift");
        cageIntake = hwMap.get(DcMotor.class, "cage_intake");
        pivotArm = hwMap.get(DcMotor.class, "pivot_arm");

        // Set drive train directions to motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Intake directions
        extendingSmall.setDirection(DcMotorSimple.Direction.FORWARD); // since pulley is connected to robot by gear
        extendingBig.setDirection(DcMotorSimple.Direction.REVERSE); // the pulley is directly attached

        // Robot lift direction
        roboLift.setDirection(DcMotorSimple.Direction.FORWARD);
        roboLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Linear extension stopping motors from slipping
        extendingBig.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendingPull.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendingSmall.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        extendingPull.setPower(0);
        extendingSmall.setPower(0);
        extendingBig.setPower(0);
        pivotArm.setPower(0);
        roboLift.setPower(0);
        cageIntake.setPower(0);

        //Initialize all servo
        markerDrop = hwMap.get(Servo.class, "marker_drop");
        liftRelease = hwMap.get(Servo.class, "lift_release");

        //TODO: add starting pos of ALL servos
        markerDrop.setPosition(0.5);
        liftRelease.setPosition(0.8); // Other pos is (0.2)

        // Sensors

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
        // Naming variables
        double heading = imu.getAngularOrientation().firstAngle;
        double roll  = imu.getAngularOrientation().secondAngle;
        double pitch = imu.getAngularOrientation().thirdAngle;
    }

    // TODO: add ALL hwmap init to method below
    public void initAutonomous(HardwareMap ahwMap) {
    }
}


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
    protected DcMotor extendingPulley = null;
    protected DcMotor extendingSprocket = null;
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

    protected double drivingWheelToCm = 560 / (9 * Math.PI); // 9 is diameter of wheel, 560 is counts per revolution

    // first value = difference between starting and ending pos
    // this accounts for num of rotations
    int pulleyToCm = (int) (560 / (5 * Math.PI)); // 5 = 5cm diameter of pulley
    double halfOfTinyPulleyCir = (0.097 * Math.PI) / 2;
    double extendingDifference = (42 * (3 * halfOfTinyPulleyCir)) - (120 * (3* halfOfTinyPulleyCir));

    int extPulleyMax = (int) (pulleyToCm * extendingDifference);  // todo value should be negative test spin
    int extSprocketMax = (int) (pulleyToCm * extendingDifference); // todo value should be negative test spin 
    int extContractPulley = (int) pulleyToCm * x; // right most value

    int pivPulleyMax = (int) (pulleyToCm * 14); // middle value    | 34cm (top) - 20cm (bottom) |
    int pivSprockedMax = (int) (pulleyToCm * 5); // left most value  | 31cm (top) - 26cm (bottom) |
    int pivContractMax = (int) (pulleyToCm * 34.5); // right most value | 40.5cm (top) - 6cm (bottom) |

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
        extendingPulley = hwMap.get(DcMotor.class, "extending_big");
        extendingSprocket = hwMap.get(DcMotor.class, "extending_small");
        extendingPull = hwMap.get(DcMotor.class, "extending_pull");
        roboLift = hwMap.get(DcMotor.class, "robo_lift");
        cageIntake = hwMap.get(DcMotor.class, "cage_intake");
        pivotArm = hwMap.get(DcMotor.class, "pivot_arm");

        // Set drive train directions to motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Linear lift directions mode
        extendingPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendingSprocket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendingPull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendingPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendingSprocket.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendingPull.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Robot lift direction
        roboLift.setDirection(DcMotorSimple.Direction.FORWARD);
        roboLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roboLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Linear extension stopping motors from slipping
        extendingPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendingPull.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendingSprocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // undo brake mode from auto on drive motors
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        extendingPull.setPower(0);
        extendingSprocket.setPower(0);
        extendingPulley.setPower(0);
        pivotArm.setPower(0);
        roboLift.setPower(0);
//        cageIntake.setPower(0);

        //Initialize all servo
//        markerDrop = hwMap.get(Servo.class, "marker_drop");
        liftRelease = hwMap.get(Servo.class, "lift_release");

        //TODO: add starting pos of ALL servos
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
// Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        // Drive train
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        extendingPulley = hwMap.get(DcMotor.class, "extending_big");
        extendingSprocket = hwMap.get(DcMotor.class, "extending_small");
        extendingPull = hwMap.get(DcMotor.class, "extending_pull");
        roboLift = hwMap.get(DcMotor.class, "robo_lift");
        cageIntake = hwMap.get(DcMotor.class, "cage_intake");
        pivotArm = hwMap.get(DcMotor.class, "pivot_arm");

        // Set drive train directions to motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Linear lift directions mode
        extendingPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendingSprocket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendingPull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendingPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendingSprocket.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendingPull.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Robot lift direction
        roboLift.setDirection(DcMotorSimple.Direction.FORWARD);
        roboLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roboLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Linear extension stopping motors from slipping
        extendingPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendingPull.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendingSprocket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        extendingPull.setPower(0);
        extendingSprocket.setPower(0);
        extendingPulley.setPower(0);
        pivotArm.setPower(0);
        roboLift.setPower(0);
//        cageIntake.setPower(0);

        //Initialize all servo
//        markerDrop = hwMap.get(Servo.class, "marker_drop");
        liftRelease = hwMap.get(Servo.class, "lift_release");

        //TODO: add starting pos of ALL servos
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

        // driving stops
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}


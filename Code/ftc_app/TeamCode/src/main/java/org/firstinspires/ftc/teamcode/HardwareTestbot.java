package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareTestbot
{

    /* Public OpMode members. */
    public static DcMotor  driveMotorFrontLeft   = null;
    public static DcMotor  driveMotorFrontRight  = null;
    public static DcMotor  driveMotorBackLeft  = null;
    public static DcMotor  driveMotorBackRight  = null;
    public static DcMotor  leftFlywheelMotor = null;
    public static DcMotor  rightFlywheelMotor = null;
    public static DcMotor  throatMotor = null;
    public static DcMotor   yogiYodaForceBear = null;
    public static Servo rightBaconator = null;
    public static Servo leftBaconator = null;
    // public static Servo    baconGetter = null;
    public static ColorSensor baconSensor = null;
    public static ModernRoboticsI2cGyro sandwichSensor   = null;                    // Additional Gyro device

    //public static Gamepad gamepad1 = new Gamepad();

    public static double turnScale = 0.5;
    public static double VyScale = 0.5;
    public static double VxScale = 0.5;
    public static double a = 17.35/2;
    public static double b = 14.35/2;

    // public static final double MID_SERVO       =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareTestbot(){
/*
        driveMotorFrontLeft = hwMap.dcMotor.get("drive_motor_front_left");
        driveMotorFrontRight = hwMap.dcMotor.get("drive_motor_front_right");
        driveMotorBackLeft = hwMap.dcMotor.get("drive_motor_back_left");
        driveMotorBackRight = hwMap.dcMotor.get("drive_motor_back_left");

        driveMotorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveMotorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveMotorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveMotorBackRight.setDirection(DcMotor.Direction.REVERSE);
        */

    }

     /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors & Sensors
        baconSensor = hwMap.colorSensor.get("bacon_sensor");
        sandwichSensor = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("sandwich_sensor");

        // baconGetter = hwMap.servo.get("bacon_getter");

        driveMotorFrontLeft   = hwMap.dcMotor.get("drive_motor_front_left");
        driveMotorFrontRight   = hwMap.dcMotor.get("drive_motor_front_right");
        driveMotorBackLeft   = hwMap.dcMotor.get("drive_motor_back_left");
        driveMotorBackRight   = hwMap.dcMotor.get("drive_motor_back_right");
        leftFlywheelMotor = hwMap.dcMotor.get("left_flywheel_motor");
        rightFlywheelMotor = hwMap.dcMotor.get("right_flywheel_motor");
        throatMotor = hwMap.dcMotor.get("throat_motor");
        yogiYodaForceBear = hwMap.dcMotor.get("lift_motor");

        rightBaconator = hwMap.servo.get("right_baconator");
        leftBaconator = hwMap.servo.get("left_baconator");

        driveMotorFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveMotorFrontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        driveMotorBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveMotorBackRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftFlywheelMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFlywheelMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        throatMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        driveMotorFrontLeft.setPower(0);
        driveMotorFrontRight.setPower(0);
        driveMotorBackLeft.setPower(0);
        driveMotorBackRight.setPower(0);
        leftFlywheelMotor.setPower(0);
        rightFlywheelMotor.setPower(0);
        throatMotor.setPower(0);
        yogiYodaForceBear.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
/*

    public static void mecanumDrive(Gamepad gamepad1) {

        driveMotorFrontRight.setPower(
                -gamepad1.left_stick_y - -gamepad1.right_stick_x + turnScale * (a + b)
        );

        driveMotorFrontLeft.setPower(
                -gamepad1.left_stick_y + -gamepad1.right_stick_x - turnScale * (a + b)
        );

        driveMotorBackLeft.setPower(
                -gamepad1.left_stick_y - -gamepad1.right_stick_x - turnScale * (a + b)
        );

        driveMotorBackRight.setPower(
                -gamepad1.left_stick_y + -gamepad1.right_stick_x + turnScale * (a + b)
        );

    }

    public static void tankDrive(Gamepad gamepad1) {

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        driveMotorFrontLeft.setPower(-gamepad1.left_stick_y);
        driveMotorFrontRight.setPower(-gamepad1.right_stick_y);
        driveMotorBackRight.setPower(-gamepad1.right_stick_y);
        driveMotorBackLeft.setPower(-gamepad1.left_stick_y);

    }

*/
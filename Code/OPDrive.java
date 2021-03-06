package org.firstinspires.ftc.teamcode;

/**
 * Created by 5815-Disgruntled on 10/13/2016.
 */

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.HardwarePushbot;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test Opmode2", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class OPDrive extends LinearOpMode {

    //public Gamepad gamepad1 = new Gamepad();
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Public OpMode members. */
    public static DcMotor  driveMotorFrontLeft   = null;
    public static DcMotor  driveMotorFrontRight  = null;
    public static DcMotor  driveMotorBackLeft  = null;
    public static DcMotor  driveMotorBackRight  = null;

    //public static Gamepad gamepad1 = new Gamepad();

    public static double turnScale = 0.5;
    public static double a = 14.513/2;
    public static double b = 13.858/2;

    public static final double MID_SERVO       =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        driveMotorFrontLeft = hwMap.dcMotor.get("drive_motor_front_left");
        driveMotorFrontRight = hwMap.dcMotor.get("drive_motor_front_right");
        driveMotorBackLeft = hwMap.dcMotor.get("drive_motor_back_left");
        driveMotorBackRight = hwMap.dcMotor.get("drive_motor_back_left");

        driveMotorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveMotorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveMotorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveMotorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //opMode code goes here

            if( !gamepad1.right_bumper ) {


                //HardwarePushbot.tankDrive(gamepad1);
                driveMotorFrontLeft.setPower(-gamepad1.left_stick_y);
                driveMotorFrontRight.setPower(-gamepad1.right_stick_y);
                driveMotorBackRight.setPower(-gamepad1.right_stick_y);
                driveMotorBackLeft.setPower(-gamepad1.left_stick_y);
                telemetry.addData("Debug", "In tank drive");
                telemetry.update();

            } else {

                //HardwarePushbot.mecanumDrive(gamepad1);
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
                telemetry.addData("Debug", "In mecanum drive");
                telemetry.update();

            }



            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}


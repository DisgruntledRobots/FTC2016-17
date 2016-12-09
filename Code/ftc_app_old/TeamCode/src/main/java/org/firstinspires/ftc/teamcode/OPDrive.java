package org.firstinspires.ftc.teamcode;

/**
 * Created by 5815-Disgruntled on 10/13/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Test Opmode2", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class OPDrive extends LinearOpMode {

    //public Gamepad gamepad1 = new Gamepad();
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    HardwareTestbot roberto = new HardwareTestbot();
    public static int gatePosition = 1;

    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        roberto.init(hardwareMap);

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

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
            telemetry.addData("Gamepad2 launcher power", gamepad2.left_stick_y);
            telemetry.update();


            // launcher and intake motor power control
            roberto.leftFlywheelMotor.setPower(gamepad2.left_stick_y);
            roberto.rightFlywheelMotor.setPower(gamepad2.left_stick_y);
            roberto.throatMotor.setPower(gamepad2.right_stick_y);

            // mecanum drive if right bumper is held, and tank drive if not
            if( gamepad1.right_bumper ) {

                roberto.driveMotorFrontRight.setPower(
                        -gamepad1.left_stick_y - -gamepad1.left_stick_x + gamepad1.right_stick_x * (roberto.a + roberto.b)
                );

                roberto.driveMotorFrontLeft.setPower(
                        -gamepad1.left_stick_y + -gamepad1.left_stick_x - gamepad1.right_stick_x * (roberto.a + roberto.b)
                );

                roberto.driveMotorBackLeft.setPower(
                        -gamepad1.left_stick_y - -gamepad1.left_stick_x - gamepad1.right_stick_x * (roberto.a + roberto.b)
                );

                roberto.driveMotorBackRight.setPower(
                        -gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x * (roberto.a + roberto.b)
                );
                telemetry.addData("Debug", "In mecanum drive");
                telemetry.update();

            } else {

                roberto.driveMotorFrontLeft.setPower(-gamepad1.left_stick_y);
                roberto.driveMotorFrontRight.setPower(-gamepad1.right_stick_y);
                roberto.driveMotorBackRight.setPower(-gamepad1.right_stick_y);
                roberto.driveMotorBackLeft.setPower(-gamepad1.left_stick_y);
                telemetry.addData("Debug", "In tank drive");
                telemetry.update();
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}


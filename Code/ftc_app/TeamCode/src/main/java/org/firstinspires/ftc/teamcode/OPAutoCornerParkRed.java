package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the roberto Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Test OPAutoCornerParkRed", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class OPAutoCornerParkRed extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTestbot roberto   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        roberto.init(hardwareMap);

        // Send telemetry message to signify roberto waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roberto.driveMotorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                roberto.driveMotorFrontLeft.getCurrentPosition(),
                roberto.driveMotorFrontRight.getCurrentPosition(),
                roberto.driveMotorBackRight.getCurrentPosition(),
                roberto.driveMotorBackLeft.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*Step through each leg of the path,
        Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderStrafeDrive(DRIVE_SPEED, (double)15);*/

        encoderDrive(DRIVE_SPEED,60,60,10);
        turnLeft((float)30.5);
        encoderDrive(DRIVE_SPEED,50,50,10);

//        roberto.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
//        roberto.rightClaw.setPosition(0.0);
//        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,double leftInches, double rightInches,double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = roberto.driveMotorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = roberto.driveMotorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            // newRightTarget = roberto.driveMotorBackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            // newLeftTarget = roberto.driveMotorBackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            roberto.driveMotorFrontLeft.setTargetPosition(newLeftTarget);
            roberto.driveMotorFrontRight.setTargetPosition(newRightTarget);
            roberto.driveMotorBackRight.setTargetPosition(newRightTarget);
            roberto.driveMotorBackLeft.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            roberto.driveMotorFrontLeft.setPower(Math.abs(speed));
            roberto.driveMotorFrontRight.setPower(Math.abs(speed));
            roberto.driveMotorBackRight.setPower(Math.abs(speed));
            roberto.driveMotorBackLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (       roberto.driveMotorFrontLeft.isBusy() &&
                            roberto.driveMotorFrontRight.isBusy()&&
                            roberto.driveMotorBackRight.isBusy() &&
                            roberto.driveMotorBackLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        roberto.driveMotorFrontLeft.getCurrentPosition(),
                        roberto.driveMotorFrontRight.getCurrentPosition(),
                        roberto.driveMotorBackRight.getCurrentPosition(),
                        roberto.driveMotorBackLeft.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            roberto.driveMotorFrontLeft.setPower(0);
            roberto.driveMotorFrontRight.setPower(0);
            roberto.driveMotorBackRight.setPower(0);
            roberto.driveMotorBackLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void shootBall(double speed, double timeoutS){
        if (opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS)){
                roberto.leftFlywheelMotor.setPower(speed);
                roberto.rightFlywheelMotor.setPower(speed);
                roberto.throatMotor.setPower(-speed);
            }

            roberto.leftFlywheelMotor.setPower(0);
            roberto.rightFlywheelMotor.setPower(0);

        }
    }

    public void encoderStrafeDrive(double speed,double strafeDistance) {
        //strafeDistance should be positive for left and negative for right

      /*  roberto.driveMotorFrontRight.setPower(strafeDistance);

        roberto.driveMotorFrontLeft.setPower(-strafeDistance);

        roberto.driveMotorBackLeft.setPower(strafeDistance);

        roberto.driveMotorBackRight.setPower(-strafeDistance);
        */

        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = roberto.driveMotorFrontLeft.getCurrentPosition() + (int) (strafeDistance * COUNTS_PER_INCH);
            // newTarget = roberto.driveMotorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            // newTarget = roberto.driveMotorBackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            // newTarget = roberto.driveMotorBackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            roberto.driveMotorFrontLeft.setTargetPosition(-newTarget);
            roberto.driveMotorFrontRight.setTargetPosition(newTarget);
            roberto.driveMotorBackRight.setTargetPosition(-newTarget);
            roberto.driveMotorBackLeft.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            roberto.driveMotorFrontLeft.setPower(Math.abs(speed));
            roberto.driveMotorFrontRight.setPower(Math.abs(speed));
            roberto.driveMotorBackRight.setPower(Math.abs(speed));
            roberto.driveMotorBackLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (roberto.driveMotorFrontLeft.isBusy() &&
                            roberto.driveMotorFrontRight.isBusy() &&
                            roberto.driveMotorBackRight.isBusy() &&
                            roberto.driveMotorBackLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", strafeDistance);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        roberto.driveMotorFrontLeft.getCurrentPosition(),
                        roberto.driveMotorFrontRight.getCurrentPosition(),
                        roberto.driveMotorBackRight.getCurrentPosition(),
                        roberto.driveMotorBackLeft.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.

            }

            // Stop all motion;
            roberto.driveMotorFrontLeft.setPower(0);
            roberto.driveMotorFrontRight.setPower(0);
            roberto.driveMotorBackRight.setPower(0);
            roberto.driveMotorBackLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void turnLeft(float inches) {

        /*
            call encoderDrive to turn to the left
            Order of parameters:
            double speed,double leftInches, double rightInches,double timeoutS
        */
        try {

            encoderDrive(TURN_SPEED, -inches, inches, 10.0);  // S2: turn towards the first beacon

        } catch (Exception ex) {

            ex.printStackTrace();
            return;

        }


    }


    public void turnRight(float inches) {

        /*
            call encoderDrive to turn to the right
        */
        try {

            encoderDrive(TURN_SPEED, inches, -inches, 10.0);  // S2: turn towards the first beacon

        } catch (Exception ex) {

            ex.printStackTrace();
            return;

        }

    }



}
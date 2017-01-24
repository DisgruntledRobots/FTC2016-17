package org.firstinspires.ftc.teamcode;

/**
 * Created by 5815-Disgruntled on 1/24/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OPEpisode1Point5_ShootParticles extends LinearOpMode {


    /* Declare OpMode members. */
    HardwareTestbot roberto   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() throws InterruptedException{

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

        //Insert movement code here
        shootBall(DRIVE_SPEED, 10);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void encoderDrive(double speed,double leftInches, double rightInches,double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                roberto.throatMotor.setPower(1);
            }

            roberto.leftFlywheelMotor.setPower(0);
            roberto.rightFlywheelMotor.setPower(0);
            roberto.throatMotor.setPower(0);

        }
    }
}

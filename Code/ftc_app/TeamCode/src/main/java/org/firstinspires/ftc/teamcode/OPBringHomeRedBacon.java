package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 5815-Disgruntled on 1/24/2017.
 */

@Autonomous(name="OPBringHomeRedBacon", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class OPBringHomeRedBacon extends LinearOpMode{
    /* Declare OpMode members. */
    HardwareTestbot roberto   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static int initGray                        = 0;

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

//        roberto.racistSensor.enableLed(true);
        roberto.baconSensor.enableLed(true);

//        lessThanWhite = roberto.racistSensor.red() + roberto.racistSensor.blue() + roberto.racistSensor.green();

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

        doMovement();


    telemetry.addData("Path", "Complete");
    telemetry.update();
}
    public void encoderDrive(double speed,double leftInches, double rightInches,double timeoutS) throws InterruptedException {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

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
            newFrontLeftTarget = roberto.driveMotorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH * 3/2);
            newFrontRightTarget = roberto.driveMotorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH * 3/2);
            newBackRightTarget = roberto.driveMotorBackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = roberto.driveMotorBackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            roberto.driveMotorFrontLeft.setTargetPosition(newFrontLeftTarget);
            roberto.driveMotorFrontRight.setTargetPosition(newFrontRightTarget);
            roberto.driveMotorBackRight.setTargetPosition(newBackRightTarget);
            roberto.driveMotorBackLeft.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            roberto.driveMotorFrontLeft.setPower(Math.abs(speed));
            roberto.driveMotorFrontRight.setPower(Math.abs(speed));
            roberto.driveMotorBackRight.setPower(Math.abs(speed * 2/3));
            roberto.driveMotorBackLeft.setPower(Math.abs(speed * 2/3));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (       roberto.driveMotorFrontLeft.isBusy() &&
                            roberto.driveMotorFrontRight.isBusy()&&
                            roberto.driveMotorBackRight.isBusy() &&
                            roberto.driveMotorBackLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newBackRightTarget,
                        newBackLeftTarget, newBackRightTarget);
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

    public void doMovement() throws InterruptedException{

        //get initial color sensor value
        initGray = roberto.racistSensor.red() + roberto.racistSensor.green() + roberto.racistSensor.blue();

//        * forward from wall

        encoderDrive(DRIVE_SPEED, 20, 20, 10);

//        * shoot

        shootBall(-DRIVE_SPEED, 3);

//        * forward to align with wall

        encoderDrive(DRIVE_SPEED, 23, 23, 10);

//        * 90 degrees left

        encoderDrive(TURN_SPEED, -22, 22, 10);

//        * forward to hit wall

        encoderDrive(DRIVE_SPEED, 48, 48, 10);

//        * 90 degrees right

        encoderDrive(TURN_SPEED, 22, -22, 10);

//        * forward/backward to match white line
        notGray();

//        * forward to next white line

        encoderDrive(DRIVE_SPEED, 20, 20, 10);

//        * 45 degrees right

        encoderDrive(DRIVE_SPEED, 12, -12, 10);

//        * find boundary

        while(opModeIsActive()){
            //I'm beginning to become suspicious of all these opModeIsActive() loops
        }

//        * 90 degrees right

        encoderDrive(DRIVE_SPEED, 22, -22, 10);

//        * forward to cap ball

        encoderDrive(DRIVE_SPEED, 40, 40, 10);

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

    public boolean isStillGray() {

        boolean isWhite = false;
        int rgbValue = roberto.racistSensor.red() + roberto.racistSensor.green() + roberto.racistSensor.blue();
        while(opModeIsActive()){

            if( Math.abs(rgbValue - initGray) > 20 ) {

                isWhite = false;

            } else {

                isWhite = true;

            }

        }

        return isWhite;
    }

    }


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 5815-Disgruntled on 2/15/2017.
 */

@Autonomous(name="OPBringHomeBlueBacon", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class OPBringHomeBlueBacon extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareTestbot roberto   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static int initGray                        = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        //rangeInit();
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

        roberto.racistSensor.enableLed(true);
        roberto.baconSensor.enableLed(true);

        roberto.sandwichSensor.calibrate();

        //get initial color sensor value
        initGray = roberto.racistSensor.red() + roberto.racistSensor.green() + roberto.racistSensor.blue();

        while (!isStarted()) {
            telemetry.addData("Red: ",roberto.baconSensor.red());
            telemetry.addData("Blue: ",roberto.baconSensor.blue());
            telemetry.addData("Init Gray: ", initGray);
            telemetry.addData("Racist Val: ", roberto.racistSensor.red() + roberto.racistSensor.green() + roberto.racistSensor.blue());
            telemetry.addData("Heading: ",roberto.sandwichSensor.getHeading());
            telemetry.update();
            idle();
        }
        roberto.sandwichSensor.resetZAxisIntegrator();

        // Wait for the game to start (driver presses PLAY)
        // waitForStart();


        //Insert movement code here

        doMovement();


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*public void rangeInit() {
        RANGE1 = hardwareMap.i2cDevice.get("xc_sensor");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
    }

    public void readRangeCM() {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("ODS", range1Cache[1] & 0xFF);
    }*/

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
            newFrontLeftTarget = roberto.driveMotorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = roberto.driveMotorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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

    public void encoderStrafe(double speed, double timeoutS) throws InterruptedException{
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)){
            roberto.driveMotorFrontRight.setPower(-speed);
            roberto.driveMotorFrontLeft.setPower(speed);
            roberto.driveMotorBackLeft.setPower(-speed);
            roberto.driveMotorBackRight.setPower(speed);
        }

        roberto.driveMotorFrontLeft.setPower(0);
        roberto.driveMotorFrontRight.setPower(0);
        roberto.driveMotorBackRight.setPower(0);
        roberto.driveMotorBackLeft.setPower(0);

    }

    public void doMovement() throws InterruptedException{

//        * forward from wall

        encoderDrive(DRIVE_SPEED, -20, -20, 10);

//        * shoot

        shootBall(-DRIVE_SPEED, 2);

//        * 45 degrees right

        encoderDrive(TURN_SPEED, 11, -11, 10);

//        * forward to align with wall

        encoderDrive(DRIVE_SPEED, -21, -21, 10);

//        * 45 degrees left

        encoderDrive(TURN_SPEED, 11, -11, 10);

//        * forward to near corner vortex

        encoderDrive(DRIVE_SPEED, -24, -24, 10);

//        * 90 degrees right

        encoderDrive(TURN_SPEED, -24, 24, 10);

        encoderDrive(DRIVE_SPEED,-4,-4,10);

//        * strafe into wall
        encoderStrafe(-TURN_SPEED,1.8);

//        * back away from wall - work in progress
        //roberto.XCountrySensor.getUltrasonicLevel();



//        * forward/backward to match white line

        roberto.driveMotorFrontRight.setPower(-0.2);
        roberto.driveMotorFrontLeft.setPower(-0.2);
        roberto.driveMotorBackLeft.setPower(-0.2);
        roberto.driveMotorBackRight.setPower(-0.2);

        while(opModeIsActive()){

            if (!isStillGray()){
                roberto.driveMotorFrontRight.setPower(0);
                roberto.driveMotorFrontLeft.setPower(0);
                roberto.driveMotorBackLeft.setPower(0);
                roberto.driveMotorBackRight.setPower(0);
                break;
            }
        }


//        * strafe right to bacon

        encoderStrafe(-TURN_SPEED, 0.6);

//        * pause for 5 seconds

        int count = 0;
        while(opModeIsActive()){
            if(count > 5){
                break;
            }
            telemetry.addData("Red: ",roberto.baconSensor.red());
            telemetry.addData("Blue: ",roberto.baconSensor.blue());
            telemetry.addData("Count: ", count);
            telemetry.update();
            sleep(1000);
            count++;
        }

//        * determine color and position to score

        if(roberto.baconSensor.red() > roberto.baconSensor.blue()){
            encoderStrafe(-TURN_SPEED, 0.5);
            encoderStrafe(TURN_SPEED, 1);
        }
        else if(roberto.baconSensor.blue() > roberto.baconSensor.red()){
            encoderDrive(-TURN_SPEED, 5, 5, 5);
            encoderStrafe(-TURN_SPEED, 0.5);
            encoderStrafe(TURN_SPEED, 1);
        }

//        * forward to next white line

//        * forward halfway to next white line
        encoderDrive(-TURN_SPEED, -6, -6, 10);

//        * strafe into wall and strafe away to straighten robot
        encoderStrafe(-TURN_SPEED,1);
        encoderStrafe(TURN_SPEED,0.5);

//        * all the way to next white line
        encoderDrive(-TURN_SPEED, -4, -4, 10);

        roberto.driveMotorFrontRight.setPower(-0.4);
        roberto.driveMotorFrontLeft.setPower(-0.4);
        roberto.driveMotorBackLeft.setPower(-0.4);
        roberto.driveMotorBackRight.setPower(-0.4);

        while(opModeIsActive()){

            if (!isStillGray()){
                roberto.driveMotorFrontRight.setPower(0);
                roberto.driveMotorFrontLeft.setPower(0);
                roberto.driveMotorBackLeft.setPower(0);
                roberto.driveMotorBackRight.setPower(0);
                break;
            }
        }

//        * pause for 5 seconds

        count = 0;
        while(opModeIsActive()){
            if(count > 5){
                break;
            }
            telemetry.addData("Red: ",roberto.baconSensor.red());
            telemetry.addData("Blue: ",roberto.baconSensor.blue());
            telemetry.addData("Count: ", count);
            telemetry.update();
            sleep(100);
            count++;
        }

//        * determine color and position to score

        if(roberto.baconSensor.red() > roberto.baconSensor.blue()){
            encoderStrafe(-TURN_SPEED, 0.5);
            encoderStrafe(TURN_SPEED, 1);
        }
        else if(roberto.baconSensor.blue() > roberto.baconSensor.red()){
            encoderDrive(-TURN_SPEED, 5, 5, 5);
            encoderStrafe(-TURN_SPEED, 0.5);
            encoderStrafe(TURN_SPEED, 1);
        }
//        * 45 degrees right

        encoderDrive(DRIVE_SPEED, -12, 12, 10);

//        * find boundary

        while(opModeIsActive()){
            //I'm beginning to become suspicious of all these opModeIsActive() loops
//             Stop whining and fix the darn thing instead!!!!!!
        }

//        * 90 degrees right

        encoderDrive(DRIVE_SPEED, -22, 22, 10);

//        * forward to cap ball

        encoderDrive(DRIVE_SPEED, -40, -40, 10);

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

        boolean isGray = true;
        int rgbValue = roberto.racistSensor.red() + roberto.racistSensor.green() + roberto.racistSensor.blue();
        telemetry.addData("Init Gray: ", initGray);
        telemetry.addData("Racist Val: ", rgbValue);
        telemetry.update();
        if ((rgbValue > initGray * 2) && opModeIsActive()){

            isGray = false;

        } else {

            isGray = true;

        }

        return isGray;
    }

    // left turn is negative angle
    // right turn is positive angle
    public void sandwichTurn(double speed, double angle, double timeoutS)  throws InterruptedException{

        roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roberto.driveMotorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        roberto.sandwichSensor.resetZAxisIntegrator();
        if(angle > 0) {
            roberto.driveMotorFrontLeft.setPower(speed);
            roberto.driveMotorFrontRight.setPower(-speed);
            roberto.driveMotorBackLeft.setPower(speed);
            roberto.driveMotorBackRight.setPower(-speed);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (roberto.sandwichSensor.getHeading() < angle)) {
                telemetry.addData("heading: ", roberto.sandwichSensor.getHeading());
                telemetry.update();
            }
        }
        else if(angle < 0){
            roberto.driveMotorFrontLeft.setPower(-speed);
            roberto.driveMotorFrontRight.setPower(speed);
            roberto.driveMotorBackLeft.setPower(-speed);
            roberto.driveMotorBackRight.setPower(speed);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (roberto.sandwichSensor.getHeading() > 360 + angle)) {
                telemetry.addData("heading: ", roberto.sandwichSensor.getHeading());
                telemetry.update();
            }
        }

        roberto.driveMotorFrontLeft.setPower(0);
        roberto.driveMotorFrontRight.setPower(0);
        roberto.driveMotorBackLeft.setPower(0);
        roberto.driveMotorBackRight.setPower(0);

        roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontLeftTarget = roberto.driveMotorFrontLeft.getCurrentPosition() + moveCounts;
            newFrontRightTarget = roberto.driveMotorFrontRight.getCurrentPosition() + moveCounts;
            newBackLeftTarget = roberto.driveMotorBackLeft.getCurrentPosition() + moveCounts;
            newBackRightTarget = roberto.driveMotorBackRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            roberto.driveMotorFrontLeft.setTargetPosition(newFrontLeftTarget);
            roberto.driveMotorFrontRight.setTargetPosition(newFrontRightTarget);
            roberto.driveMotorBackLeft.setTargetPosition(newBackLeftTarget);
            roberto.driveMotorBackRight.setTargetPosition(newBackRightTarget);

            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            roberto.driveMotorFrontLeft.setPower(speed);
            roberto.driveMotorFrontRight.setPower(speed);
            roberto.driveMotorBackLeft.setPower(speed);
            roberto.driveMotorBackRight.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (roberto.driveMotorFrontLeft.isBusy() && roberto.driveMotorFrontRight.isBusy() &&
                            roberto.driveMotorBackLeft.isBusy() && roberto.driveMotorBackRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                roberto.driveMotorFrontLeft.setPower(leftSpeed);
                roberto.driveMotorFrontRight.setPower(rightSpeed);
                roberto.driveMotorBackLeft.setPower(leftSpeed);
                roberto.driveMotorBackRight.setPower(rightSpeed);

                /* Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      roberto.leftMotor.getCurrentPosition(),
                        roberto.rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
                */
            }

            // Stop all motion;
            roberto.driveMotorFrontLeft.setPower(0);
            roberto.driveMotorFrontRight.setPower(0);
            roberto.driveMotorBackLeft.setPower(0);
            roberto.driveMotorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            roberto.driveMotorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roberto.driveMotorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        roberto.driveMotorFrontLeft.setPower(0);
        roberto.driveMotorFrontRight.setPower(0);
        roberto.driveMotorBackLeft.setPower(0);
        roberto.driveMotorBackRight.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        roberto.driveMotorFrontLeft.setPower(leftSpeed);
        roberto.driveMotorFrontRight.setPower(rightSpeed);
        roberto.driveMotorBackLeft.setPower(leftSpeed);
        roberto.driveMotorBackRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - roberto.sandwichSensor.getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}

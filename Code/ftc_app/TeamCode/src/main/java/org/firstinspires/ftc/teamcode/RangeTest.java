package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 5815-Disgruntled on 2/15/2017.
 */

@TeleOp(name="RangeTest", group="Linear Opmode")

public class RangeTest extends LinearOpMode {
    HardwareTestbot roberto   = new HardwareTestbot();
    private ElapsedTime runtime = new ElapsedTime();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    //I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    //public I2cDevice RANGE1;
    //public I2cDeviceSynch RANGE1Reader;

    //@Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("RANGE1: ", roberto.RANGE1);
        telemetry.addData("RANGE1Reader: ", roberto.RANGE1Reader);
        telemetry.update();

        //RANGE1 = hardwareMap.i2cDevice.get("xc_sensor");
        //RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        //RANGE1Reader.engage();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            range1Cache = roberto.RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("ODS", range1Cache[1] & 0xFF);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            idle();
        }
    }
}
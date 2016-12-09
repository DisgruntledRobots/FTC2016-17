package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by 5815-Disgruntled on 10/28/2016.
 /

public class OPTestColor {

    import android.app.Activity;
    import android.graphics.Color;
    import android.view.View;

    import com.qualcomm.ftcrobotcontroller.R;
    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.ColorSensor;

    /*
     *
     * This is an example LinearOpMode that shows how to use
     * a Modern Robotics Color Sensor.
     *
     * The op mode assumes that the color sensor
     * is configured with a name of "color sensor".
     *
     * You can use the X button on gamepad1 to toggle the LED on and off.
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */
   @TeleOp(name = "Test MR Color", group = "LinearOpMode")
   @Disabled
    public class OPTestColor extends LinearOpMode {

        HardwareTestbot roberto = new HardwareTestbot();    // Hardware Device Object

        @Override
        public void runOpMode() throws InterruptedException {

            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F,0F,0F};

            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;

            // get a reference to the RelativeLayout so we can change the background
            // color of the Robot Controller app to match the hue detected by the RGB sensor.
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

            // bPrevState and bCurrState represent the previous and current state of the button.
            boolean bPrevState = false;
            boolean bCurrState = false;

            // bLedOn represents the state of the LED.
            boolean bLedOn = true;

            // get a reference to our ColorSensor object.
            // roberto.baconSensor = hardwareMap.colorSensor.get("bacon_sensor");

            // Set the LED in the beginning
            //roberto.baconSensor.enableLed(bLedOn);

            // wait for the start button to be pressed.
            waitForStart();

            // while the op mode is active, loop and read the RGB data.
            // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
            while (opModeIsActive()) {

                // check the status of the x button on either gamepad.
                bCurrState = gamepad1.x;

                // check for button state transitions.
                if ((bCurrState == true) && (bCurrState != bPrevState))  {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    //roberto.baconSensor.enableLed(bLedOn);
                }

                // update previous state variable.
                bPrevState = bCurrState;

                // convert the RGB values to HSV values.
                /*Color.RGBToHSV(roberto.baconSensor.red() * 8,
                                roberto.baconSensor.green() * 8,
                                roberto.baconSensor.blue() * 8,
                                hsvValues);


                // send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", roberto.baconSensor.alpha());
                telemetry.addData("Red  ", roberto.baconSensor.red());
                telemetry.addData("Green", roberto.baconSensor.green());
                telemetry.addData("Blue ", roberto.baconSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                // change the background color to match the color detected by the RGB sensor.
                // pass a reference to the hue, saturation, and value array as an argument
                // to the HSVToColor method.
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });
                */

                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
        }
    }

/*
Copyright 2024 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class TeleOpA extends LinearOpMode {
    private Blinker control_Hub;
    private IMU imu;
    private DcMotor test_motor;
    private Servo intakeServo;
    private Servo clawServo;
    private DcMotor test_motorB;
    private DcMotor d_motorL;
    private DcMotor d_motorR;
    private DcMotor armMotor;
    private DcMotor clawMotor;
    private double currentIntakePosition;
    private double currentClawPosition;


    @Override
    public void runOpMode() {
        // Assign components to varibles
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        imu = hardwareMap.get(IMU.class, "imu");
        d_motorL= hardwareMap.get(DcMotor.class, "d_motor_l");
        d_motorR = hardwareMap.get(DcMotor.class, "d_motor_r");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        clawMotor = hardwareMap.get(DcMotor.class, "claw_motor");

        // Initalise global varibles
        currentIntakePosition = 0.0;
        currentClawPosition = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            
            // Call all functions to control robot
            DriveTrainControl();
            IntakeControl();
            ClawControl();
            ArmMotorControl();
            ClawMotorControl();
            
            telemetry.update();

        }
    }
    
    private void DriveTrainControl() {
        float lm = 0f; // Final varible for what left motor power should be
        float rm = 0f; // Final varible for what the right motor power shoule be
        float sx = gamepad1.left_stick_x; // x-axis on left stick, goes from -1 to 1
        float ntc; // Constant to make power negative if left trigger is pulled.
        float tl = gamepad1.left_trigger; // left trigger varible (float of how far pulled from 0.0 -> 1.0)
        float tr = gamepad1.right_trigger; // right trigger varible
        // Find of which trigger is pulled further and set NTC to be negative if the left trigger is pulled further
        float tp= Math.max(tl ,tr); 
        if (tr > tl) {
            ntc = 1.0f;
        } else {
            ntc = -1.0f;
        }
        
    
        if ((0 > sx) && (sx >= -1)) { // IF x-axis on stick is negative
            lm = tp*(sx+1.0f)*ntc;
            rm = tp*ntc;
        } else if ((1 >= sx) && (sx > 0)) { // if x-axis on stick is positive
            lm = tp*ntc;
            rm = -1.0f*tp*(sx-1.0f)*ntc;
        } else { // if x-axis on stick is 0
            lm = tp*ntc;
            rm = tp*ntc;
        }
        // telemetry/troubleshooting data
        telemetry.addData("lt", gamepad1.left_trigger);
        telemetry.addData("rt", gamepad1.right_trigger);
        
        telemetry.addData("lm", lm);
        telemetry.addData("rm", rm);
        
        // Set motor powers and make left-motor power negative so it goes in the oppisite direction
        d_motorL.setPower((double)(lm * -1.0f));
        d_motorR.setPower((double)rm);
        
    }
    
    private void IntakeControl() {        

        if (gamepad1.dpad_up) {
            intakeServo.setPosition(1);
        } else if (gamepad1.dpad_down) {
            intakeServo.setPosition(0);
        }
        
        
    }
    
    private void ClawControl() {
        telemetry.addData("cP", (double) clawServo.getPosition());
        double openPos = 0;
        double closePos = 0.1;
        if (gamepad1.dpad_left) {
            clawServo.setPosition(closePos);
        } else if (gamepad1.dpad_right) {
            clawServo.setPosition(openPos);
        }
        
    }
    
    private void ArmMotorControl() {
        if (gamepad1.right_stick_y > 0) {
            armMotor.setPower(gamepad1.right_stick_y);
            
        } else if (gamepad1.right_stick_y < 0) {
            armMotor.setPower(gamepad1.right_stick_y);
        } else {
            armMotor.setPower(0);
        }
    }
    
    private void ClawMotorControl() {
        if (gamepad1.right_stick_x != 0) {
            clawMotor.setPower(0.5*gamepad1.right_stick_x);   
        } else {
            clawMotor.setPower(0);
        }
    }
    
    
}

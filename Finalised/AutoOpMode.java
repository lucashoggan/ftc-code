/*
Copyright 2025 FIRST Tech Challenge Team FTC

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous

public class AutoOpMode extends OpMode {
    private Blinker control_Hub;
    private DcMotor arm_motor;
    private DcMotor claw_motor;
    private Servo claw_servo;
    private DcMotor d_motor_l;
    private DcMotor d_motor_r;
    private IMU imu;
    private Servo intake_servo;
    private ElapsedTime runtime = new ElapsedTime();
    private double curTime;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // Setup hardware
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        claw_motor = hardwareMap.get(DcMotor.class, "claw_motor");
        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        d_motor_l = hardwareMap.get(DcMotor.class, "d_motor_l");
        d_motor_r = hardwareMap.get(DcMotor.class, "d_motor_r");
        imu = hardwareMap.get(IMU.class, "imu");
        intake_servo = hardwareMap.get(Servo.class, "intake_servo");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Set left drivetrain motor to reverse direction
        d_motor_l.setDirection(DcMotorSimple.Direction.REVERSE);
        
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("time", runtime.milliseconds());
        // Set current time in program
        curTime = 1000.0;
        curTime = DriveDistance(curTime, 100, 1.0);
        
        
        telemetry.update();
    }
    
    public double Drive(double start, double end, double speed_l, double speed_r) {
        if ((runtime.milliseconds() >= start) && (runtime.milliseconds() <= end)) { // Between start time and end time, set drivetrain motors to desired power
            d_motor_l.setPower(speed_l);
            d_motor_r.setPower(speed_r);
        } 
        if ((runtime.milliseconds() > end) && (runtime.milliseconds() <= end+100.0)) { // Between the end and end+100 milliseconds, hault the motors
            d_motor_l.setPower(0);
            d_motor_r.setPower(0);
        }
        return end+100.0; // return the time for when current process is finished
    }
    
    public void HaultDrive(double time) {
        if ((runtime.milliseconds() >= time) && (runtime.milliseconds() <= time+100)) { // between start and end time, stop motors
            d_motor_l.setPower(0);
            d_motor_r.setPower(0);
        }
    }
    
    public double DriveDistance(double start, double distance, double power) {
        double deltaTime = CalcDistanceTime(distance, power); // Get the estimated time to travel the desired distance.
        if ((runtime.milliseconds() >= start) && (runtime.milliseconds() <= start+deltaTime)) { // Set power of motor for desired time
            d_motor_l.setPower(power);
            d_motor_r.setPower(power);
        } if ((runtime.milliseconds() > start+deltaTime) && (runtime.milliseconds() <= start+deltaTime+100)) { // After driving has ended, hault motors
            d_motor_l.setPower(0);
            d_motor_r.setPower(0);
        }
        return start+deltaTime+100; // Return estimated end time
    }
    
    public double TankTurn(double startTime, double deltaTime, double speed, boolean antiClock) {
        // If clockwise desied, set motors to desired speed for desired time but make right motor spinn backwards
        if (antiClock == false) {
            Drive(startTime, startTime+deltaTime, speed, (-1.0 * speed));
        } else { // Opposite of clockwise
            Drive(startTime, startTime+deltaTime, (-1.0*speed), speed);
        }
        return startTime+deltaTime; // Return estimated end time.
    }
    
    public double CalcDistanceTime(double distance, double motorPower) {
        // 0.168 cm/ms for 1.0 motor power
        double realisedSpeed = 0.168*motorPower; // calc speed considering motorpower
        return distance/realisedSpeed; // Calculate estimated time nessesary to cover distance.
    }
    
    public double RightTurnClock(double startTime) {
        return TankTurn(startTime, 2000, 0.25, false);
    }
    
    public double RightTurnAntiClock(double startTime) {
        return TankTurn(startTime, 2000.0, 0.25, true);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    };
}

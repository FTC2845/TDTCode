package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

@TeleOp(name = "SkyStone Main Teleop")
public class SkystoneMainTeleop1 extends OpMode {
    /* Declare OpMode members. */
    private static final int BASE_LIFT = 250;
    private static final int TOP_LIFT = 7120;
    private int currentLiftLevel = -1;
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor slider;
    private Servo draggerZero;
    private Servo draggerOne;
    private Servo gripperLeft;
    private Servo gripperRight;
    private Servo swivel;
    private Servo gripUp;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        slider = hardwareMap.dcMotor.get("slider");
        leftLift  = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        
        draggerZero = hardwareMap.servo.get("draggerZero");
        draggerOne = hardwareMap.servo.get("draggerOne");
        gripperLeft = hardwareMap.servo.get("gripperLeft");
        gripperRight = hardwareMap.servo.get("gripperRight");
        swivel = hardwareMap.servo.get("swivel");
        gripUp = hardwareMap.servo.get("gripUp");
        
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        int positionLeft = leftLift.getCurrentPosition();
        int positionRight = rightLift.getCurrentPosition();
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runTime.reset();
    }
    
    @Override
    public void loop(){
        updateDrive();
        updateDragger();
        updateGripper();
        updateGripperHeadLifter();
        updateSwivel();
        updateLift();
        updateSlider();
        updateSliderGrabberHeight();
    
        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

  //      telemetry.addData("leftLift Encoder: ", positionLeft);
  //      telemetry.addData("rightLift Encoder:", positionRight);
   //     telemetry.addData("slider Encoder:", positionSlider);
   //     telemetry.addData("slider power:", slidePower);
   //     telemetry.addData("drive:", drive);
  
  // if (gamepad2.left_bumper){
    
   //    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);      
   //     rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
   //     leftLift.setTargetPosition(200);
   //     rightLift.setTargetPosition(200);
  //  }
 
        
    }
    
    //******Driving Code***********
    public void updateDrive() {
        telemetry.addLine("Drive System");
        //mecanum running
        double[] driveSpeeds = getDriveSpeeds(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
      
        if (gamepad1.left_bumper){
            driveSpeeds[0] /= 0.7;
            driveSpeeds[1] /= 0.7;
            driveSpeeds[2] /= 0.7;
            driveSpeeds[3] /= 0.7;
        } else {
            driveSpeeds[0] *= 0.4;
            driveSpeeds[1] *= 0.4;
            driveSpeeds[2] *= 0.4;
            driveSpeeds[3] *= 0.4;
        }
        
        leftFront.setPower(driveSpeeds[0]);        
        rightFront.setPower(driveSpeeds[1]);
        leftBack.setPower(driveSpeeds[2]);
        rightBack.setPower(driveSpeeds[3]);
        
        telemetry.addData("Motor 1", driveSpeeds[0]);
        telemetry.addData("Motor 2", driveSpeeds[1]);
        telemetry.addData("Motor 3", driveSpeeds[2]);
        telemetry.addData("Motor 4", driveSpeeds[3]);
    }
    
    //****Dragger code for moving foundation*******
    public void updateDragger() {
        if (gamepad1.a) {
            setDraggerOpen(true);
            telemetry.addData("Dragger Status", "Open");
        } else if (gamepad1.b) {
            setDraggerOpen(false);
            telemetry.addData("Dragger Status", "Closed");
        }
    }
        
    // **************Gripper code to grab the skystones***********
    public void updateGripper() {
        if (gamepad2.b) {
            setGripperOpen(true);
             telemetry.addData("Gripper Status", "Open");
      } else if (gamepad2.a) {
            setGripperOpen(false);
             telemetry.addData("Gripper Status", "Closed");
       }
    }
    
    // *********Swivel code for the gripper head*************
    public void updateSwivel() {
        if (gamepad2.dpad_left) {
           setGripperOrientation(true);
           telemetry.addData("Swivel", "Vertical");
        } else if (gamepad2.dpad_right) {
           setGripperOrientation(false);
           telemetry.addData("Swivel", "Horizontal");
       }
    }
    
    // *********code for the gripper head lifter *************
    public void updateGripperHeadLifter() {
        if (gamepad2.x) {
           setGripperDown(true); 
           telemetry.addData("Griper Head", "Down");
        } else if (gamepad2.y) {
           setGripperDown(false);
           telemetry.addData("Griper Head", "Up");
       }
    }
    
    // *********Lift Code*******
    public void updateLift() {
        telemetry.addLine("Lift System");
        
        double drive = -gamepad2.left_stick_y;
        double[] speeds = setLiftSpeed(drive, gamepad2.left_bumper);
        
        telemetry.addData("Left Speed", speeds[0]);
        telemetry.addData("Right Speed", speeds[1]);
    }
    
    //**********Slider Code*************
    public void updateSlider() {
    // *******CODE FOR THE SLIDER ARM****************

        double slidePower  =  -gamepad2.right_stick_y;
        int positionSlider = slider.getCurrentPosition();
        
         // if the lift goes out to 3800, stop extending
        
        if ((positionSlider>3800)&&(slidePower>0)){
        slidePower = 0;
        }
        
        if ((positionSlider>3600)&&(slidePower>0)){
        slidePower = (slidePower*.1);
        }
        
        // When slder goes back to zero, stop it from going further so we dont get slack
        
        if ((!gamepad2.right_bumper)){
        if ((positionSlider<=1)&&(slidePower<0)){
        slidePower = 0;
        }
        }
        if ((positionSlider<=200)&&(slidePower<0)){
        slidePower = (slidePower*.1);
        }
        slider.setPower(slidePower);
    }
    
    //  ***** auto set grabber height and slider at start ***** 
    public void updateSliderGrabberHeight() {
      
       if (gamepad2.right_bumper){
           gripUp.setPosition(.65); // DONT MESS WITH THESE VALUES 
           
      }
    }
    
    public double[] getDriveSpeeds(float x, float y, float rotation) {
        double r = Math.hypot(x, -y);
        double robotAngle = Math.atan2(-y, x) - Math.PI / 4;;
        
        double v1 = r * Math.cos(robotAngle) + rotation;
        double v2 = r * Math.sin(robotAngle) - rotation;
        double v3 = r * Math.sin(robotAngle) + rotation;
        double v4 = r * Math.cos(robotAngle) - rotation;
        
        return new double[] { v1, v2, v3, v4 };
    }
    
    public void setDraggerOpen(boolean isOpen) {
        if (isOpen) {
            draggerOne.setPosition(.28); // DONT MESS WITH THESE VALUES
            draggerZero.setPosition(.82);
        } else {
            draggerOne.setPosition(.92);
            draggerZero.setPosition(.15);
        }
    }
    
    public void setGripperOpen(boolean isOpen) {
        if (isOpen) {
            gripperLeft.setPosition(.53); // DONT MESS WITH THESE VALUES .53
            gripperRight.setPosition(.56); // DONT MESS WITH THESE VALUES
        } else {
            gripperLeft.setPosition(1); // DONT MESS WITH THESE VALUES .53
            gripperRight.setPosition(1); // DONT MESS WITH THESE VALUES
        }
    }
    
    public void setGripperOrientation(boolean isVertical) {
        if (isVertical) {
            swivel.setPosition(0.21);
        } else {
            swivel.setPosition(0.56);
        }
    }
    
    public void setGripperDown(boolean isDown) {
        if (isDown) {
           gripUp.setPosition(.2); // DONT MESS WITH THESE VALUES 
        } else {
           gripUp.setPosition(1);
       }
    }
    
    public double[] setLiftSpeed(double speed, boolean override) {
        // *********code for the lift assembly *************
        /*   Our strategy is to keep the two motors in relative synchronization by slowing down one of the motors if 
            one motor runs faster than the other and the other is running behind.  This is to keep the assembly running true and not sticking or tilting.
            First we are going to get the value from the left stick and the encoder values from the two lift motors.
        */
        int positionLeft = leftLift.getCurrentPosition();
        int positionRight = rightLift.getCurrentPosition();
        
        //**********Safety Stop Section!!********************
        // ************ Setting the limits of the lift, both up and down***************

        // if the lift is higher than 7120, stop lifting
        // Or when lift goes back to zero, stop it from going further
        if ((positionLeft > TOP_LIFT && speed > 0) ||
            (positionLeft < BASE_LIFT && speed <= 0 && !override)) {
            speed = 0;
        }
        
        double[] speeds = { speed, speed };
        
        // Once we get the encoder values, we can compare the encoder values to create the difference between the two
        // motors' positions.
        
        int LR_enc_diff = positionLeft - positionRight;
        int RL_enc_diff = positionRight - positionLeft;
        
       // We have two scenarios:
       //   1. Raising the lift full power or lowering the lift at full power
       //   2. Raising the lift at less than full power, or lowering the lift at less than full power
       // Why these are different is because it is superior to use one motor encoder as the master, and the other
       // a slave.  Our strategy to keep the "lagging" motor in sync is to apply more power to the slave motor if 
       // it is lagging behind, and less power to the slave if it is running faster than the master.
       // This is easy if the lift is running in Scenario #2 above, because we can apply more or less power to the other 
       // motor.  But in Scenario #1, we are already at max power (value = 1) for the Master, so we cannot apply power.  
       // So for Scenario #1, we will simply use a differential approach, slow down whichever motor is running faster.
       // This is not good for Scenario #2, because we can get oscillation of the lift which is hard to control.
            
        // Here is the code for Scenario #1 
      
        if ((speed >.9) || (speed < -.9)) {
            // If the magnitude of difference between the encoder values is less than 3,
            // we apply power evenly to the two motors
            // However, if the left has gone too far (more than 3) compared to the right
            // we slow down the left motor relative to the right motor
            
            if (LR_enc_diff >= 3){
                speeds[0] = speed - (LR_enc_diff * .003);
            }
            
            // Conversely,  if the Right has gone too far (more than 5) compared to the left
            // we slow down the right motor relative to the right motor
            if (RL_enc_diff >= 3){
                speeds[1] = speed - (RL_enc_diff*.003);
            }
        } else { // ********** THis is scenario #2***************       
            
            // If the magnitude of difference between the encoder values is less than 3,
            // we apply power evenly to the two motors
            // However, if the left has gone too far (more than 3) compared to the right
            // we speed up the right motor going up, and slow it if going down
            if (LR_enc_diff>=5){
                speeds[1] = speed + (LR_enc_diff * .005);
            }
        
            // Conversely,  if the Right has gone too far (more than 5) compared to the left
            // we speed up the left motor going up, and slow it if going down
            if (RL_enc_diff >= 5) {
                speeds[1] = speed - (RL_enc_diff * .005);
            }
        }
        
        leftLift.setPower(speeds[0]);
        rightLift.setPower(speeds[1]);
        return speeds;
    }
}


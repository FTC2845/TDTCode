package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import java.util.Stack;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
@TeleOp(name = "FreightFrenzyTeleopOne")

public class FreightFrenzyTeleopOne extends OpMode {

 //********** Creat hardware instances 

    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private DcMotorEx armLeft;
    private DcMotorEx armRight;
    private DcMotorEx duckRoller1;
    private DcMotorEx duckRoller2;   
    private Servo gripperServoRight;
    private Servo gripperServoLeft;
    
    public double robotAngle;

  //*********variables for arm drive and control

    private int targetArmPosition = 0;
    private int currentArmPosition = 0;
    private double armPower =0;
    private double armError =0; 
  

   //******** Constants Collection!!!***************
  
    public double  gripperServoRightClosed= .95;
    public double  gripperServoLeftClosed= 0; 
    public double  gripperServoRightOpen= .8;    
    public double  gripperServoLeftOpen= .1;
    private int    armTicks = 10;
          
  
  //****************************************************
    
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        armLeft = hardwareMap.get(DcMotorEx.class, "armLeft");
        armRight = hardwareMap.get(DcMotorEx.class, "armRight");
    
        duckRoller1 = hardwareMap.get(DcMotorEx.class, "duckRoller1");
        duckRoller2 = hardwareMap.get(DcMotorEx.class, "duckRoller2");
        gripperServoRight = hardwareMap.servo.get("gripperServoRight");
        gripperServoLeft = hardwareMap.servo.get("gripperServoLeft");
        

       
        
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        armLeft.setDirection(DcMotorEx.Direction.FORWARD);
        armRight.setDirection(DcMotorEx.Direction.REVERSE);    
        duckRoller1.setDirection(DcMotorEx.Direction.REVERSE);
        duckRoller2.setDirection(DcMotorEx.Direction.REVERSE);
  
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
  
        armLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  
        armLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
  
         
        duckRoller1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        duckRoller2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
  
        duckRoller1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        duckRoller2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
         imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotAngle = angles.firstAngle;
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
        runtime.reset();
 
    }
    
    @Override
    public void loop(){
        
  
        updateDrive();
        updateGripperServos();
        updateDuckRoller();
        updateArmPosition();
        updateArmDrive();    

     }  
        
    

    
    
    //******Driving Code***********

  
    public void updateDrive() {
    
    
     //new mecanum drive

    
     double[] driveSpeeds = getDriveSpeeds(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
      
     if (gamepad1.left_bumper){
        driveSpeeds[0] /= 0.75;
        driveSpeeds[1] /= 0.75;
        driveSpeeds[2] /= 0.75;
        driveSpeeds[3] /= 0.75;
        } else {
        driveSpeeds[0] *= 0.45;
        driveSpeeds[1] *= 0.45;
        driveSpeeds[2] *= 0.45;
        driveSpeeds[3] *= 0.45;
        }
        
        leftFront.setPower(driveSpeeds[0]);        
        rightFront.setPower(driveSpeeds[1]);
        leftBack.setPower(driveSpeeds[2]);
        rightBack.setPower(driveSpeeds[3]);
    
        }
    
    
    public void updateArmPosition(){
        
        if(gamepad2.right_stick_y > .5){ 
            
             targetArmPosition = targetArmPosition + armTicks;
    
        }
        
        if(gamepad2.right_stick_y < -.5){ 
  
             targetArmPosition = targetArmPosition - armTicks;

            
        }
        if(gamepad2.dpad_up){
            currentArmPosition = 0;
        }
        if(gamepad2.dpad_left){
            targetArmPosition = currentArmPosition + 15;
        }
        if(gamepad2.dpad_right){
            targetArmPosition = currentArmPosition - 15;
        }
        // high position
            if(gamepad2.x){ 
            
             targetArmPosition = 1100;
        }
        // middle position
            if(gamepad2.a){ 
            
             targetArmPosition = 1280;
            }
            // low position
            if(gamepad2.b){ 
            
             targetArmPosition = 1380;
            }
            // home position
            if(gamepad2.y){ 
            
             targetArmPosition = 0;
            }
    }
    
   
  
     public void updateArmDrive(){
    
         currentArmPosition = armRight.getCurrentPosition();  // Get the position of the arm in ticks 
    
        armError = targetArmPosition - currentArmPosition;  // determine the error in ticks between our current arm position and where it should be
        
        if (armError > 5 || armError < -5){              // Hysteresis band to provide damping/braking zone to reduce oscillations
            
        armPower = ((armError * .03)+ .1);
      
                 if (armPower > .4){
                 armPower = .4;
                 }
                 if (armPower < -.4){
                 armPower = -.4;
                 } 
        }
        else {
      
         armPower = 0;
      
        } // end of if-else
    
      armLeft.setPower(armPower);
      armRight.setPower(armPower);   
    
    }
  
  
     
    public void updateDuckRoller(){
      //update the powers polarity for each side  
        if(gamepad2.left_trigger > 0.5){             
            duckRoller1.setPower(.85);
            duckRoller2.setPower(.85);
        }     
        else if(gamepad2.right_trigger > 0.5){             
            duckRoller1.setPower(-.85);
            duckRoller2.setPower(-.85);     
        }
        else {
             duckRoller1.setPower(0);
             duckRoller2.setPower(0);            
        } 
        
          
        
      
    }

  
    public void updateGripperServos(){
        if(gamepad2.left_bumper){ 
            gripperServoRight.setPosition(gripperServoRightClosed);      // close Right gripper servo   
            gripperServoLeft.setPosition(gripperServoLeftClosed);}      // close left gripper servo
        if(gamepad2.right_bumper){
            gripperServoRight.setPosition(gripperServoRightOpen);      // open Right gripper servo   
            gripperServoLeft.setPosition(gripperServoLeftOpen);}      // open left gripper servo
       
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

}
    

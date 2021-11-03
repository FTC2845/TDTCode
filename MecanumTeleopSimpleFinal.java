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
@TeleOp(name = "mecanumTeleSimple")

public class MecanumTeleopSimple extends OpMode {

  

    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
   private DcMotorEx arm;
   private DcMotorEx shooter;
   private DcMotorEx intake;
   private Servo shooterServo;
   private Servo wobbleServo;
    private int loopCount = 0;
    private double loopTime= 0;
    private double averageLoopTime= 0;
    private double maxLoopTime = 0;
    public double newElapsedTime = 0;
    public int armPosition = 0; 
    public long setTime;
    public double offAngle;
    public double robotAngle;
    public Orientation angles;
    public double remainingticks = 0;
    public double ycorrection = 0;
    public double rotation = 0;
    public double pwr;
    public double leftpwr=0;
    public double rightpwr=0;
    public double pfactor = (.002);
    public int      powerShotMoveStep;
    public boolean  powerShotMoveActive;
    public boolean  powerShotShoot;   
    public boolean  shooterPhaseOne;  
    public boolean  shooterPhaseTwo;
    public boolean  shooterPhaseThree; 
    public boolean  shooterPhaseFour; 
    public boolean  powerShotActive;
    public double ticks;
    public double timeDelayStart;
    public double timeDelayCheck;
    public double power;
    public double bias;
    public double targetAngle;
    
    
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        wobbleServo = hardwareMap.servo.get("wobbleServo");
        shooterServo = hardwareMap.servo.get("shooterServo");
        
        
        
       
        
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        // arm.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
       leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
       leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
       rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
       rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); 
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
         imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.loggingEnabled = false;
     //   imu.initialize(imuParameters);
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
        
     if (!powerShotActive){   
        updateDrive();
       // updateArm();
        //updateServo();
        //updateShooter();
        //updateIntake();
     }  
        
       // autoPower();
        
        // newElapsedTime = runtime.milliseconds();
        
        // loopTime =  newElapsedTime - oldElapsedTime;
        // loopCount = loopCount +1;
        // averageLoopTime = ( runtime.milliseconds() / loopCount);
        // if (loopTime > maxLoopTime){
        //     maxLoopTime = loopTime;
        // }
        // oldElapsedTime = newElapsedTime;
    
        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

   //     telemetry.addData("drive:", drive);

    }
    
    //******Driving Code***********
    public void updateDrive() {
    
    
     //new mecanum drive
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
        rightBack.setPower(driveSpeeds[3]);}
        
    public void updateShooter(){
        if(gamepad2.right_trigger >= 0.5){
        shooter.setVelocity(2500);} 
        else if(gamepad2.left_stick_button){ //"slow mode"
            shooter.setVelocity(2250);  
        }
    else{
            shooter.setPower(0);   }

    }
    
public void updateIntake(){
    if(gamepad2.right_bumper){
        intake.setPower(1);
    }
    else if(gamepad2.left_bumper){
        intake.setPower(-1);
    }
    else{intake.setPower(0);}
}
    
    public void updateArm(){
        
        if(gamepad2.right_stick_y > .5){ 
            
             armPosition = armPosition + 3;
                //  Move Arm to pick up
            arm.setTargetPosition(armPosition);
            arm.setPower(0.7);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        
        if(gamepad2.right_stick_y < -.5){ 
             armPosition = armPosition - 3;
                //  Move Arm to go down
            arm.setTargetPosition(armPosition);
             arm.setPower(0.7);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
      //gamepad2.y == arm 90º
    }   
    public void sleepy(long time){
        while(System.currentTimeMillis() - time < 200 ) {
         //Will only run after .2 seconds, and will only run once
        }
        shooterServo.setPosition(0.43); //puts it back
        
    }
    public void updateServo(){
        if(gamepad2.b){ 
        wobbleServo.setPosition(0.03);}      // close wobble servo to release wobble
        if(gamepad2.a){
        wobbleServo.setPosition(.465);}      // Open wobble servo to release wobble
        if (gamepad2.x){
            shooterServo.setPosition(.3); // moves it out
            //faux sleep statement
            setTime = System.currentTimeMillis();
            sleepy(setTime);
            
        }
        
    }   
   public void autoPower(){
    
        if (gamepad1.b){


            double bias = 0;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robotAngle = angles.firstAngle;
    //        double targetAngle = robotAngle;
 
 // Set up the main loop state flags
 
            powerShotActive= true;          // true if the main loop is still active
            powerShotMoveActive =true;      // true if the movement is in process
            powerShotMoveStep = 1 ;         // sets a flag for each Powershot pin
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robotAngle = angles.firstAngle;
            targetAngle =robotAngle;
 
          shooter.setVelocity(2350);
            
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
             
            }
            
        
    if (powerShotActive){
        
        
     // This section sets up the movement in ticks from the wall for each powershot pin   
        
            if(powerShotMoveStep == 1){
                ticks = 2000;
            }
            if(powerShotMoveStep ==2 ){
                ticks = 3000;
            }
            if(powerShotMoveStep ==3 ){
                ticks = 4000;
            }
   
   
   // loop to strafe the robot
   
           
        if (powerShotMoveActive){
            double power = 0.4;
             ticks = ticks * -1;             
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robotAngle = angles.firstAngle;
  
            if ((targetAngle ==180) && (robotAngle < 1)){
                    robotAngle = (360+ robotAngle);  }
            offAngle = targetAngle - robotAngle;
            remainingticks = ticks - rightBack.getCurrentPosition();
 //     Hysteresis to prevent overshoot
            if (offAngle < 0.3 && offAngle > -0.3) {
                rotation =0;
             }
            if (offAngle >= 0.3 || offAngle <= -0.3) {
                        if (offAngle >=.3) {
                        rotation = -(offAngle * 0.025 + 0);
                        }
                        if (offAngle <=-.3) {
                      rotation = -(offAngle * 0.025 + 0);
                        }
                }
                
                
                
            if (remainingticks < -500){
                 pwr = power +.2;   
                } 
            else{
              pfactor = (-power / -500);
                pwr = ((-remainingticks * pfactor) + .2);
                }  
            if ((rightBack.getCurrentPosition() > -500) && (power > .3)) {
                pfactor = (-power / -500);            
                pwr = ((-rightBack.getCurrentPosition() * pfactor) + .45);
                if (pwr > (power+.2)){
                    pwr = power +.2;  
                }
            }
            
            
            
        bias =  -(rightFront.getCurrentPosition()*.001);  // keep robot from drifting from center line
                double r = Math.hypot(pwr, -ycorrection);
                double robotAngleDrive = Math.atan2(-ycorrection, pwr) - Math.PI / 4;;
                double v1 = r * Math.cos(robotAngleDrive) + rotation + bias;
                double v2 = r * Math.sin(robotAngleDrive) - rotation + bias;
                double v3 = r * Math.sin(robotAngleDrive) + rotation + bias;
                double v4 = r * Math.cos(robotAngleDrive) - rotation + bias;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4); 
 
    // This section tests for each movement to reach its target position, then sets up the shooter loop   
                
               if(ticks > rightBack.getCurrentPosition()){
                powerShotMoveActive = false;
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                powerShotShoot = true;
                shooterPhaseOne = true;
            }

}



            if(powerShotShoot){  //loop to shoot each ring
                
                
              if (shooterPhaseOne){         // Phase One moves the servo towards the ring and resets the time clock

                shooterServo.setPosition(0.3);
                runtime.reset();  
                shooterPhaseOne = false;
                shooterPhaseTwo = true;

              }
              if (shooterPhaseTwo){                     // Phase Two is simply a delay loop to allow time for 
                timeDelayCheck =runtime.milliseconds(); // the servo in Phase One to move
                if (timeDelayCheck>300){
                shooterPhaseTwo = false;
                shooterPhaseThree = true;  
            
                 }
              }
              if (shooterPhaseThree){                   // Phase Three initiates the move of the servo back     
                shooterServo.setPosition(0.43);
                shooterPhaseThree = false;
                shooterPhaseFour = true;
                runtime.reset(); 
              }
              if (shooterPhaseFour){                   // Phase Four is a time delay to move the servo back 
                if (runtime.milliseconds()>300){        // then exits the shooting routine back to moving
                   shooterPhaseFour = false;
                   powerShotShoot = false;
                   powerShotMoveStep = (powerShotMoveStep +1); // This section increments the step in the movement
                   powerShotMoveActive =true;
       
                   if (powerShotMoveStep >3){               // After we shoot the third ring, we need to stop and exit
                      powerShotActive = false;
                      leftFront.setPower(0);
                      rightFront.setPower(0);
                      leftBack.setPower(0);
                      rightBack.setPower(0);
                        shooter.setVelocity(0);
                      
                      }
                 }
              }
        
        
            }
     

   

                 
           
            telemetry.addData("shooterPhaseOne:", shooterPhaseOne);            
              telemetry.addData("shooterPhase2:", shooterPhaseTwo);            
              telemetry.addData("shooterPhase3:", shooterPhaseThree);
               telemetry.addData("timeDelayStart:", timeDelayStart);
               telemetry.addData("timeDelayCheck:", timeDelayCheck);
                 telemetry.addData("runtime:", System.currentTimeMillis());             
            telemetry.update();  
     
     
            
    }// Button used to exit the powershot routine
        
        if(gamepad1.a){
            powerShotActive = false;
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
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
}
    

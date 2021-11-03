package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import java.util.Stack;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.List;
@Autonomous(name = "blue auto ", group = "Linear Opmode")
public class autonomousBlue extends LinearOpMode  {

    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private BNO055IMU imu;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx arm;
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo wobbleServo;
    private Servo shooterServo;
    ModernRoboticsI2cRangeSensor rightDistance;
    ModernRoboticsI2cRangeSensor frontDistance;
 
    
    int positionLeft;
    int positionleftFront;

    int positionrightFront;

    int positionRight;
    public int rings = 0;
    public boolean ringsNotFound = true;
    public boolean ringsTimeout = true;
    public int ringsTimeStart = 0;
    public int ringsIncTime = 0;
    double drive;
    public double ycorrection = 0;
    public double rotation = 0;
    public double rotationTwo = 0;
    public double rotationOne = 0;
    public double robotAngle;
    public Orientation angles;
    public double inchYOffset;
    public double frontDist;
    public double rightDist;
    public double oldTime=0;
    public double newTime=0;
    public double oldTicks=0;
    public double newTicks=0;
    public double deltaTime=0;
    public double deltaTicks=0;
    public double currentVelocity=0;
    public boolean velocityState = true;
    public double startVelocity=0;    
    public double targetVelocity=0;
    public double oldAngle=1;
    public double newAngle=0;
    public double strafeDist;
    public double forwardDist;
    public double off_angle = 0;
    public double remainingticks = 0;
    public double  sOpen = 0.43;
    public double sClose = 0.3;
    public final double wOpen = 0.465;
    public double wClose = 0.03;
    public double setVelocity = 2350 ;
                
    public double pwr;
    public double leftpwr=0;
    public double rightpwr=0;
    public double pfactor = (.002);
     public boolean xMoveDone =false;
    public boolean yMoveDone = false;
    public double currentXposition;
    public double currentYposition;
    public double xDirection;
    public double yDirection;
    public int actualXposition = 0;
    public int actualYposition = 0;
    public int xticks;
    public int yticks;
    public boolean velocityStateY = true;   
    public double remainingYticks = 0; 
    public double targetYvelocity=0;   
    public double startYvelocity=0;  
    public double currentYvelocity=0;    
    public double deltaYTicks=0;    
    public double oldYTicks=0;
    public double newYTicks=0;  
    public double xpwr;
    public double ypwr; 

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

 
    private static final String VUFORIA_KEY =
            "AVedM9X/////AAABmepr/cyLkkrDo8488anmS8CAjidKmcpAtrUoeq+51IZ4eD8W7pJXmqk9x0LdGXS//rek1nex4IEhlcKfPf8hj1lSwWpGkbYwS/8v3Y15L1uIeBZ96H2SMin49JBdv6A/mPqjNDKxfm3wJO5mMWlE21sQz3nfobQYqmQ4o8mgAuC8EDgLBFZH8TRmvwc0lJC6ggin2MB6Tp0YZFE7kqXX1O/2K/obJ/VwQ7l2iF8VDityCRycUpUYMMhalzQlEIL1TWaN8wKBJ+LF1M5KiDe491mU848qarz129E8QvzZPAoQHKXvt5QFWsXDejBISUBXT1GRrS3Onn7DgSZpAZ8ImpQnu4rG5cls53tAvwzxT4ju";


    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

/**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }  



    public int inchesToTicks(double inches) {
        final double wheelDiameter = 2.8;
        final double distancePerRevolution = wheelDiameter * Math.PI; //Inches
        final double ticksPerRevolution = 1000;

        return (int)(((inches) / distancePerRevolution) * ticksPerRevolution);
    }
       public int inchesToTicks2(double inches) {
        final double wheelDiameter = 2.8;
        final double distancePerRevolution = wheelDiameter * Math.PI; //Inches
        final double ticksPerRevolution = 1011;

        return (int)(((inches) / distancePerRevolution) * ticksPerRevolution);
    }

     
//**********SET POWER ZERO************
public void setPowerZero(){
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
        }
//**********FORWARD AND REVERSE DRIVING**************        

    public void driveInches(double inches, double power, double targetAngle) {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        int ticks = inchesToTicks(inches);


        if (inches > 0) { // inches is greater than zero; positive
            
            velocityState = true;               // flag for setting the start velocity when doing the deceleration PID

            while ((ticks > rightFront.getCurrentPosition())&& opModeIsActive()) {
                
                
                
                
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                      
                  }
                off_angle = robotAngle- targetAngle ;
                remainingticks = ticks - rightFront.getCurrentPosition();
    
   // Code to set up the ability to measure current velocity  
                
        newTicks = rightFront.getCurrentPosition();
        newTime = runtime.milliseconds();
        deltaTicks = oldTicks-newTicks;
        deltaTime = oldTime -newTime;
        currentVelocity = (deltaTicks/deltaTime);
        oldTime = newTime;
        oldTicks = newTicks;
   
 
   
 // For short moves, under 20 inches, at low power
 
        if (ticks <= 1000){
            
          pfactor = (power/ ticks);
                pwr = ((remainingticks * pfactor) + .2);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher speeds       
        else {
                if (remainingticks >1500){   // When the robot is in the middle of long move, give full power
                 pwr = power +.2;   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityState){                     // This flag is TRUE when the routine starts
                startVelocity = currentVelocity;        // Capture the current velocity at beginning of deceleration
                velocityState = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }


                targetVelocity = (remainingticks/1500)*startVelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                pwr = ((currentVelocity - targetVelocity)* -.2) +.20;   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
// This is the acceleration PID method.  At applied power levels greater than .3, the wheels tend 
// to spin if you apply the power immediately.  This method applies a ramping up of power from 0 
// ticks to 1000 ticks.  The initial applied power starts at .25 and ramps up to the target power.
// 


                if ((rightFront.getCurrentPosition() < 1000) && (power > .3)) {
                        pfactor = (power/ 1000);
                        pwr = ((rightFront.getCurrentPosition() * pfactor) + .25);
                 
                        if (pwr > (power+.15)){ // the above code has the possibility of applying 
                                                // more than the target power, so this section 
                                                // limits the applied power to the target power
                   
                          pwr = power +.15;  
                  
                        }
                 
                }       
 
        }
 
                
   //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
                leftpwr = pwr;
                rightpwr = pwr;
                                     
             }
            
  //    Turn robot back to heading if the current heading is off from desired heading          
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        
                        leftpwr = pwr + (off_angle * 0.025+0);
                        rightpwr = pwr - (off_angle * 0.025 + 0);
                        
                           
                        
                      
 
                }    
               
         
                leftFront.setPower(leftpwr);
                rightFront.setPower(rightpwr);
                leftBack.setPower(leftpwr);
                rightBack.setPower(rightpwr);
     


            }
            
            setPowerZero();
         

        }

     if (inches < 0) { //inches is less than zero 


            velocityState = true;               // flag for setting the start velocity when doing the deceleration PID
      
            while ((ticks < rightFront.getCurrentPosition())&& opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                    if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                      
                  }
                off_angle = robotAngle-targetAngle  ;              
                remainingticks = ticks - rightFront.getCurrentPosition();
               
         newTicks = rightFront.getCurrentPosition();
        newTime = runtime.milliseconds();
        deltaTicks = oldTicks-newTicks;
        deltaTime = oldTime -newTime;
        currentVelocity = (deltaTicks/deltaTime);
        oldTime = newTime;
        oldTicks = newTicks;
   
 
   
 // For short moves, under 20 inches, at low power. When you are moving slow and the distance is short,
 // the robot does not have a lot of inertia.  Therefore, we don't need to apply negative power in the
 // PID function to apply braking.  This PID is simply a proportional function with a small amount 
 // of feedforward.
 
        if (ticks >= -1000){
            
          pfactor = (power/ ticks);
                pwr = ((-remainingticks * pfactor) - .25);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher power.  For these moves, the 
 // robot has a lot of inertia, and the PID method needs to apply reverse power to apply braking versus
 // simply decreasing applied power.  To accomplish this, the PID routine measures the velocity of the 
 // robot, calculates the target velocity, then applies power to have the robot follow the velocity target.
 
 
        else {
                if (remainingticks < -1500){   // When the robot is in the middle of long move, give full power
                 pwr = -power -.25;   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityState){                     // This flag is TRUE when the routine starts
                startVelocity = currentVelocity;        // Capture the current velocity at beginning of deceleration
                velocityState = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }

                targetVelocity = -(remainingticks/1500)*startVelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                pwr = ((currentVelocity - targetVelocity)* -.2) -.25;   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
                if ((rightFront.getCurrentPosition() > -1000) && (-power < -.3)) {
                        pfactor = (-power/ -1000);
                        pwr = ((rightFront.getCurrentPosition() * pfactor) - .25);
                 
                        if (pwr < (-power-.15)){
                   
                          pwr = -power -.15;  
                  
                        }
                 
                }       
 
        }
 
  
  
  
         telemetry.addData("remaining ticks:", remainingticks);
         telemetry.addData("pwr:", pwr);      
         telemetry.addData("ticks:", ticks);
           telemetry.addData("current position:", rightFront.getCurrentPosition());
            telemetry.addData("pfactor:", pfactor);          
 
            telemetry.update();

 //     Hysteresis to prevent overshoot
              if (off_angle < 0.5 && off_angle > -0.5) {
                leftpwr = pwr;
                rightpwr = pwr;
             
             }
            
  //    Turn robot back to heading if the current heading is off from desired heading          
                if (off_angle >= 0.5 || off_angle <= -0.5) {
                        
                        leftpwr = pwr + (off_angle * 0.03 + 0);
                        rightpwr = pwr - (off_angle * 0.03 + 0);
                }    
                leftFront.setPower(leftpwr);
                rightFront.setPower(rightpwr);
                leftBack.setPower(leftpwr);
                rightBack.setPower(rightpwr);
            }
            
           setPowerZero();

        }
    
    }
        
    //***************LEFT RIGHT STRAFING******************


    public void strafeInches(double inches, double power, double targetAngle) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticks = inchesToTicks2(inches);
        ticks = ticks * -1;

    //      double exponentTicks =0;
    //    double exponentEncoder =0; 
    //     double accelThresholdTicks = (ticks * .67);
    //    double accelCoefficient = 0;

    double bias = 0;
        if (inches > 0) { //positive
            while ((ticks < rightBack.getCurrentPosition())&& opModeIsActive()) {
 
                 angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                    if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                      
                  }
                off_angle = targetAngle - robotAngle;
           remainingticks = ticks - rightBack.getCurrentPosition();
        //       double pwr;

 //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
            rotation =0;
                
             }
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        if (off_angle >=.3) {
                        rotation = -(off_angle * 0.02 + 0);
            
                        }
                        if (off_angle <=-.3) {
                       rotation = -(off_angle * 0.02 + 0);
            
                        }
                      
                }
                    
             //   exponentTicks = Math.pow (ticks,5);
             //   exponentEncoder = Math.pow (rightBack.getCurrentPosition(),5);
             //   double currentInchPosition = (rightBack.getCurrentPosition()* .00186);
             //   pwr=((((-power/exponentTicks)*exponentEncoder)+power)+.15);

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
                 
        bias =  -(rightFront.getCurrentPosition()*.001);
        
        
        telemetry.addData("remaining ticks:", remainingticks);
         telemetry.addData("pwr:", pwr);      
         telemetry.addData("ticks:", ticks);
           telemetry.addData("current position:", rightBack.getCurrentPosition());  
            telemetry.addData("pfactor:", rightBack.getCurrentPosition()); 
            telemetry.update();

                
                double r = Math.hypot(pwr, -ycorrection);
                double robotAngle = Math.atan2(-ycorrection, pwr) - Math.PI / 4;;

                double v1 = r * Math.cos(robotAngle) + rotation + bias ;
                double v2 = r * Math.sin(robotAngle) - rotation + bias;
                double v3 = r * Math.sin(robotAngle) + rotation + bias;
                double v4 = r * Math.cos(robotAngle) - rotation + bias;

                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);

 

            }
            setPowerZero();

        }

        if (inches < 0) { //negative

        while ((ticks > rightBack.getCurrentPosition())&& opModeIsActive()) {
            
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                    if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                      
                  }
                off_angle = targetAngle - robotAngle;
            remainingticks = ticks - rightBack.getCurrentPosition();
          //      double pwr;

 //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
            rotation =0;
                
             }
            
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        if (off_angle >=.3) {
                        rotation = -(off_angle * 0.02 + 0);
            
                        }
                        if (off_angle <=-.3) {
                       rotation = -(off_angle * 0.02 + 0);
            
                        }
                      
                }
    //   if (remainingticks >500){
    //              pwr = -power -.25;   
    //             } 
    //             else{
    //           pfactor = (power / 500);
    //             pwr = (-(remainingticks * pfactor) - .25);
         
    //             }  
    //             if ((rightBack.getCurrentPosition() < 500) && (power > .3)) {
    //                     pfactor = (power / 1000);
    //                     pwr = ((-rightBack.getCurrentPosition() * pfactor) - .20);
    //                     if (pwr < (-power-.15)){
    //                       pwr = -power -.15;  
    //                     }
                
    //              }
               pwr = -power;
               
                bias =  -(rightFront.getCurrentPosition()*.001);
        telemetry.addData("remaining ticks:", remainingticks);
         telemetry.addData("pwr:", pwr);      
         telemetry.addData("ticks:", ticks);
           telemetry.addData("current position:", rightBack.getCurrentPosition());  
            telemetry.addData("pfactor:", pfactor); 
            telemetry.update();


                double r = Math.hypot(pwr, -ycorrection);
                double robotAngle = Math.atan2(-ycorrection, pwr) - Math.PI / 4;;

                double v1 = r * Math.cos(robotAngle) + rotation + bias;
                double v2 = r * Math.sin(robotAngle) - rotation + bias;
                double v3 = r * Math.sin(robotAngle) + rotation + bias;
                double v4 = r * Math.cos(robotAngle) - rotation + bias;

                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);

 

            }
            setPowerZero();

        }

    }
    //automates shooting, shoots the amount of rings the user wants.
    public void shootRing (int j){
        
        for(int i = 0; i < j; i++){
         shooterServo.setPosition(sClose); //close
        sleep(300);
        shooterServo.setPosition(sOpen); //open
        sleep(300);
        }
        sleep(200);
      
        }
       
    
    public void driveTicksRelative(int xticks, int yticks,double xpower, double ypower,double targetAngle) {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

       
    
        xMoveDone =false;
        yMoveDone =false;
        
        // We need a flag to be able to apply power in the correct direction
        // so we use a variable to change polarity
        
        if (xticks<0){
            
            xDirection = -1;
        }
        else xDirection = 1;
        
         if (yticks<0){
            
            yDirection = -1;
        }
        else yDirection = 1;
        
    // For either straight forward back or strafe moves when the X or Y axis 
    // is zero, we need to set the move in that direction to zero to start 
    // the loop
        
        
        if (xticks==0){
          
           xMoveDone =true;  
            
        }
      
        
         if (yticks==0){
            
           yMoveDone =true; 
        }
   
        
            
            velocityState = true;  // flag for setting the start velocity when doing the deceleration PID
            velocityStateY = true;  


            while ((!xMoveDone ||!yMoveDone) && opModeIsActive()) {
                
            if ((xticks* xDirection) < (rightFront.getCurrentPosition()* xDirection)){
                xMoveDone = true;
            }    
            if ((yticks * yDirection) < (-rightBack.getCurrentPosition()* yDirection)){
                yMoveDone = true;
            }    
                
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                  }
                off_angle = robotAngle- targetAngle ;
                remainingticks = (xticks * xDirection) - (rightFront.getCurrentPosition()* xDirection);
                remainingYticks = (yticks * yDirection) -  (-rightBack.getCurrentPosition()* yDirection);
    
   // Code to set up the ability to measure current velocity  
                
        newTicks = (rightFront.getCurrentPosition()* xDirection);
        newYTicks = (-rightBack.getCurrentPosition()* yDirection);
        newTime = runtime.milliseconds();
        deltaTicks = oldTicks-newTicks;
        deltaYTicks = oldYTicks-newYTicks;
        deltaTime = oldTime -newTime;
        currentVelocity = (deltaTicks/deltaTime);
        currentYvelocity = (deltaYTicks/deltaTime);
        oldTime = newTime;
        oldTicks = newTicks;
        oldYTicks = newYTicks;
        

 
  if (!xMoveDone){
   
 // For short moves, under 10 inches
 
        if ((xticks * xDirection)  <= 1000){
            
          pfactor = (xpower/ (xticks * xDirection));
                xpwr = (((remainingticks * pfactor) + .2)* xDirection);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher speeds       
        else {
                if (remainingticks >1500){   // When the robot is in the middle of long move, give full power
                 xpwr = ((xpower+.2) * xDirection);   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityState){                     // This flag is TRUE when the routine starts
                startVelocity = currentVelocity;        // Capture the current velocity at beginning of deceleration
                velocityState = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }


                targetVelocity = (remainingticks/1500)*startVelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                xpwr = ((((currentVelocity - targetVelocity)* -.3) +.25)* xDirection);   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
// This is the acceleration PID method.  At applied power levels greater than .3, the wheels tend 
// to spin if you apply the power immediately.  This method applies a ramping up of power from 0 
// ticks to 1000 ticks.  The initial applied power starts at .25 and ramps up to the target power.
// 


                if (((rightFront.getCurrentPosition()* xDirection) < 500) && (xpower > .3)) {
                        pfactor = (xpower/ 500);
                        xpwr = ((((rightFront.getCurrentPosition()* xDirection) * pfactor) + .25)* xDirection);
                 
                        if ((xpwr* xDirection) > xpower){ // the above code has the possibility of applying 
                                                // more than the target power, so this section 
                                                // limits the applied power to the target power
                   
                          xpwr = (xpower* xDirection);  
                  
                        }
                 
                }       
 
        }
        
  }
 
   if (!yMoveDone){ 
   
        // For short moves, under 10 inches
 
        if ((yticks * yDirection) <= 500){
            
          pfactor = (ypower/ (yticks * yDirection));
                ypwr = (((remainingYticks * pfactor) + .2) * yDirection);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher speeds       
        else {
                if (remainingYticks >500){   // When the robot is in the middle of long move, give full power
                 ypwr = ((ypower +.2) * yDirection);   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityStateY){                     // This flag is TRUE when the routine starts
                startYvelocity = currentYvelocity;        // Capture the current velocity at beginning of deceleration
                velocityStateY = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }


                targetYvelocity = (remainingYticks/500)*startYvelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                ypwr = ((((currentYvelocity - targetYvelocity)* -.2) +.25) * yDirection);   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
// This is the acceleration PID method.  At applied power levels greater than .3, the wheels tend 
// to spin if you apply the power immediately.  This method applies a ramping up of power from 0 
// ticks to 1000 ticks.  The initial applied power starts at .25 and ramps up to the target power.
// 


                if (((-rightBack.getCurrentPosition() * yDirection) < 500) && (ypower > .3)) {
                        pfactor = (ypower/ 500);
                        ypwr = (((-rightBack.getCurrentPosition() * yDirection) * pfactor) + .25);
                 
                        if (ypwr > ((ypower+.15) * yDirection)){ // the above code has the possibility of applying 
                                                // more than the target power, so this section 
                                                // limits the applied power to the target power
                   
                          ypwr = ((ypower +.15) * yDirection);  
                  
                        }
                 
                }       
 
        } 
        
   }    
        
   if (xMoveDone){

        currentXposition=  (rightFront.getCurrentPosition()* xDirection);
    
        if (currentXposition < ((xticks +25)* xDirection) && currentXposition > ((xticks -25)* xDirection)) {
            xpwr =0;
        }
            
        if (currentXposition >= ((xticks +25)* xDirection) || currentXposition <= ((xticks -25)* xDirection)){    
            xpwr = ((((xticks* xDirection) - currentXposition) * .0005)* xDirection);    
        }
    }
 
    if (yMoveDone){

        currentYposition=  (-rightBack.getCurrentPosition() * yDirection);
    
        if (currentYposition < ((yticks +25) * yDirection) && currentYposition > ((yticks -25) * yDirection)) {
            ypwr =0;
        }
            
        if (currentYposition >= ((yticks +25) * yDirection) || currentYposition <= ((yticks -25) * yDirection)){    
            ypwr = ((((yticks * yDirection) - currentYposition)  * .001) * yDirection);    
                      
        }
     
 }      
        
        
        
   //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
            rotation =0;
                
             }
            
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        if (off_angle >=.3) {
                        rotation = (off_angle * 0.015 + 0);
            
                        }
                        if (off_angle <=-.3) {
                       rotation = (off_angle * 0.015 + 0);
            
                        }
                      
                }
               
              double r = Math.hypot(ypwr, xpwr);
                double robotAngle = Math.atan2(xpwr, ypwr) - Math.PI / 4;;

                double v1 = r * Math.cos(robotAngle)+ rotation  ;
                double v2 = r * Math.sin(robotAngle)- rotation ;
                double v3 = r * Math.sin(robotAngle) + rotation ;
                double v4 = r * Math.cos(robotAngle) - rotation ;

                leftFront.setPower(v1/.7);
                rightFront.setPower(v2/.7);
                leftBack.setPower(v3/.7);
                rightBack.setPower(v4/.7);

                 newTime = runtime.milliseconds();
            
               telemetry.addData("remaining ticks:", remainingticks);
         telemetry.addData("pwr:", xpwr);
          telemetry.addData("ypwr:", ypwr);        

           telemetry.addData("current position:", rightFront.getCurrentPosition());
            telemetry.addData("pfactor:", pfactor);          
            
            telemetry.update();
                
            }
            
            setPowerZero();
    
    }
    

    @Override
    public void runOpMode() {
 

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        
        rightDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightDistance");
        frontDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontDistance");
        
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
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotAngle = angles.firstAngle;
        double offAngle;
        int targetAngle;
        double inchOffset=0;

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        
        

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }    
            
        // Wait for the game to start (driver presses PLAY)
        
         wobbleServo.setPosition(0.025); 
        waitForStart();
        
        runtime.reset();                // Reset the main timer

        
        
         // run until the end of the match (driver presses STOP)
 
        if (opModeIsActive()) {
            
      double ringsTimeStart = runtime.milliseconds();  // Get the current time before entering the ring stack detect loop
        double ringsIncTime =0;          
            
            
        while (ringsNotFound && ringsTimeout) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {


                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
 
                                
                            if (recognition.getLabel()== "Single"){
                                rings= 1;
                                ringsNotFound=false;
                            }
                            if (recognition.getLabel()== "Quad"){
                                rings= 4;
                                ringsNotFound=false;
                                }       
                              telemetry.addData("rings", rings);                               
                              
                                
                      }
                     
                      telemetry.update();
                    }       // bounds of if updatedRecognitions is not null loop
                    
                }           // bounds of if tfod is not null loop


// If TensorFlow doesn't see a ring, we need a timeout function to break out of the loop after a period of time
                
                ringsIncTime = runtime.milliseconds();
                if ((ringsIncTime- ringsTimeStart)>1000
                ){   // This is the timeout value if TensorFlow doesnt see a ring
                  ringsTimeout = false;  
                  rings = 0;                
                }
                
            }               // bounds of main tfod loop
            
            
//****************************************************************************************
//*
//*     NO RING FOUND - TARGET ZONE A 
//*     
//*     This is the section of code to run when there are no rings in the stack
//*
//*
//*     
//*
//*
//*****************************************************************************************
        if (rings == 0){
        //auto plan 
       
        //auto start!
        //go forwards 68 inches
        driveInches(54,0.6, 0);
        strafeInches(-2.5,0.5,0);
        sleep(200);
        //places wobble goal
        arm.setTargetPosition(675);         //  Move Arm to place wobble on mat
        arm.setPower(.5);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(1000);                        // Give arm time to move to drop position
        wobbleServo.setPosition(wOpen);      // Open wobble servo to release wobble
        sleep(500);
        driveInches(-5,0.5, 0); //***
        shooter.setVelocity(2450);
        strafeInches(28,0.5,0);
        
     //   driveInches(3,0.5, 0);
        
        
        
         // Turn right to -180 heading
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // robotAngle =angles.firstAngle;
        robotAngle = 0;
        targetAngle = -179;
        newTime = runtime.milliseconds();
        while (!(targetAngle >=robotAngle  )) { //while target angle is NOT greater than equal to bot angle, if negative switch it :)
            double PIDpwr = ((targetAngle - robotAngle) * 0.006) - .1;
            //double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;
            double left_pwr = PIDpwr * -1 ;
            double right_pwr = PIDpwr ;
            leftFront.setPower(left_pwr);
            leftBack.setPower(left_pwr);
            rightFront.setPower(right_pwr);
            rightBack.setPower(right_pwr);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robotAngle =angles.firstAngle;
            newAngle = robotAngle;
            if ((targetAngle < -90) && (robotAngle > 10)){
                    
                  robotAngle = (robotAngle-360);  
                      
                      
                 }
            //  oldAngle = newAngle;
            //  oldTime = newTime;
            //  newAngle = robotAngle;
            //  newTime = runtime.milliseconds();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            newAngle =angles.firstAngle;
            telemetry.addData("Target Angle", targetAngle);            
            telemetry.addData("Robot Angle", robotAngle); 
            telemetry.addData("IMU Angle", newAngle);
                  telemetry.addData("PIDpower", PIDpwr);      
            telemetry.update();
            }
        setPowerZero();
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //SHOOT 
        shootRing(3);
     //   strafeInches(-2,0.7,180); 
    
    //     // Drive down to pick up next wobble
      driveInches(14, 0.6,180);
        
    //     // Alignment code********************(sideways)
    //     // We need to adjust the sideways distance the robot is away from the wall 
      rightDist = rightDistance.getDistance(DistanceUnit.INCH);
      sleep (100);
      frontDist = frontDistance.getDistance(DistanceUnit.INCH);
      sleep(100);
        telemetry.addData("frontdist", frontDist);
        telemetry.addData("rightdist", rightDist);      


      strafeDist = (rightDist - 29);
      forwardDist = (frontDist -31);
      

            telemetry.addData("strafe", strafeDist);      
            telemetry.addData("front", forwardDist);  
            telemetry.update();   
              
    strafeInches(strafeDist, 0.6,180); 
    driveInches(forwardDist, 0.4,180); //***
     
    //     // pick up wobble
      sleep(750);   
        wobbleServo.setPosition(wClose);      // close wobble servo 
     sleep(750); 
        arm.setTargetPosition(600);         //  Move Arm to pick up wobble
        arm.setPower(.8);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);                       // Give arm time to go back to zero position
        arm.setPower(.8);
        
        // back up to point to align to return
        telemetry.addData("drove forty" , -40);
        telemetry.update();
        driveInches(-45, 0.5,180); 
        
    //     // turn right to get back to where the wobble drop is
        
        // Turn right to 90 heading
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotAngle =angles.firstAngle;
        if (robotAngle < 1){
         robotAngle = (-robotAngle);  
         }
        
        targetAngle = 90;
        newTime = runtime.milliseconds();
        while (!(targetAngle >=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
         double PIDpwr = ((targetAngle - robotAngle) * 0.009) - .08;
         //double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;
         double left_pwr = PIDpwr * -1 ;
         double right_pwr = PIDpwr ;
         leftFront.setPower(left_pwr);
         leftBack.setPower(left_pwr);
         rightFront.setPower(right_pwr);
         rightBack.setPower(right_pwr);
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         robotAngle =angles.firstAngle;
         newAngle = robotAngle;
         if (robotAngle < 1){
              robotAngle = (-robotAngle);  
             }
         //    oldAngle = newAngle;
         //   oldTime = newTime;
         //   newAngle = robotAngle;
          //   newTime = runtime.milliseconds();
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         newAngle =angles.firstAngle;
         telemetry.addData("Robot Angle", robotAngle); 
         telemetry.addData("IMU Angle", newAngle); 
         telemetry.update();
         }
     
     //soft stop
     setPowerZero();
     leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     
     // drive forward to drop wobble   
     driveInches(8, 0.6,90); 
     
     // drop wobble
     arm.setTargetPosition(675);         //  Move Arm to place wobble on mat
        arm.setPower(.5);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(500);
     wobbleServo.setPosition(wOpen);      // Open wobble servo to release wobble ***
     sleep(500); 
     
     // back up on line 
     driveInches(-10, 0.6,90); 
 
    }        
 
  //****************************************************************************************
//*
//*     ONE RING FOUND - TARGET ZONE B
//*     
//*     This is the section of code to run when there is one ring in the stack
//*
//*
//*               
//*
//*
//*****************************************************************************************
 
 
   
   
     if (rings == 1){
         //auto plan
        //target zone b is 95 inches
        //the wobble goal grabber is on the left side, so you don't need to turn if it's red 
        //will need 2 turn if blue(selected)
        //go straight
        // turn if blue
        //drop wobble goal
        //turn back to starting position
        //return 17 inches 
        
        //auto start!
        //drive 52 inches
        
        driveInches(83,0.5, 0);

        // angle strafe over
        strafeInches(21,0.6, 0);
         shooter.setVelocity(2500);
        

       //drop wobble
      
      arm.setTargetPosition(675);         //  Move Arm to place wobble on mat
        arm.setPower(.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);                        // Give arm time to move to drop position
        wobbleServo.setPosition(wOpen);      // Open wobble servo to release wobble
        sleep(1000);
        driveInches(-24,0.5,0);


     


     // turn right 180 degrees
      
                 // Turn right to -180 heading
            
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  //robotAngle =angles.firstAngle;
                  robotAngle= 0;
                  targetAngle = -179;
                  newTime = runtime.milliseconds();
                  while (!(targetAngle >=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = ((targetAngle -robotAngle) * 0.006) - .13;
         //         double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;
                  double left_pwr = PIDpwr * -1 ;
                  double right_pwr = PIDpwr ;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  newAngle = robotAngle;
                 if ((targetAngle < -90) && (robotAngle > 10)){
                    
                  robotAngle = (robotAngle-360);  
                      
                      
                 }
    
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  newAngle =angles.firstAngle;
                      telemetry.addData("Robot Angle", robotAngle); 
                    telemetry.addData("IMU Angle", newAngle);
                         telemetry.addData("power", PIDpwr);               
                    
                          telemetry.update();
                  }
                setPowerZero();
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

sleep(100);


//SHOOT
shootRing(3);
//move forward 
shooter.setVelocity(2450);
intake.setPower(-1);
driveInches(18,0.5,180);
//collect 
sleep(1000);
//driveInches(-18,0.5,180);
shootRing(1);
shooter.setPower(0);
intake.setPower(0);


    
    //     // Drive down to pick up next wobble
      //driveInches(22, 0.5,180);
        
    //     // Alignment code********************(sideways)
    //     // We need to adjust the sideways distance the robot is away from the wall 
      rightDist = rightDistance.getDistance(DistanceUnit.INCH);
      sleep(100);
      frontDist = frontDistance.getDistance(DistanceUnit.INCH);
      sleep(100);
        telemetry.addData("frontdist", frontDist);
        telemetry.addData("rightdist", rightDist);      
            telemetry.update();

      strafeDist = (rightDist - 29);
      forwardDist = (frontDist -31);
      
  //      strafeDist = strafeDist * -1;//switch polarity
            telemetry.addData("strafe", strafeDist);      
            telemetry.addData("front", forwardDist);  
            telemetry.update();   
                  
    strafeInches(strafeDist, 0.6,180); 
    
        driveInches(forwardDist, 0.4,180); //***
     
    //     // pick up wobble
     sleep(750);   
        wobbleServo.setPosition(wClose);      // close wobble servo 
     sleep(750); 
        arm.setTargetPosition(550);         //  Move Arm to pick up wobble
        arm.setPower(.8);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);                       // Give arm time to go back to zero position
        arm.setPower(.8);
        
//   // back up to point to align to return
  
              driveInches(-50, 0.6,180);
  
              
//   // turn right to get back to where the wobble drop is
   
//              // Turn right to 90 heading
            
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                if (robotAngle < 1){
                    
                    robotAngle = (-robotAngle);  
                      
                      
                  }
                  targetAngle = 90;
                  newTime = runtime.milliseconds();
                  while (!(targetAngle >=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = ((targetAngle - robotAngle) * 0.009) - .09;
         //         double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;
                  double left_pwr = PIDpwr * -1 ;
                  double right_pwr = PIDpwr ;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  newAngle = robotAngle;
                    if (robotAngle < 1){
                    
                    robotAngle = (-robotAngle);  
                      
                      
                  }
             //    oldAngle = newAngle;
              //   oldTime = newTime;
              //   newAngle = robotAngle;
              //   newTime = runtime.milliseconds();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  newAngle =angles.firstAngle;
                      telemetry.addData("Robot Angle", robotAngle); 
                    telemetry.addData("IMU Angle", newAngle); 
                          telemetry.update();
                  }
                  leftFront.setPower(0);
                  leftBack.setPower(0);
                  rightFront.setPower(0);
                  rightBack.setPower(0); 
                  leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
     // strafe right to drop area 
        driveInches(-8,.6,90);
          strafeInches(12, .6,90); 
 
    // drop wobble
        
          arm.setTargetPosition(675);         //  Move Arm to place wobble on mat
        arm.setPower(.5);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(500);
     wobbleServo.setPosition(wOpen);      // Open wobble servo to release wobble
     sleep(350); 
     
    
    // strafe left to line
    
            strafeInches(-15, .8,90); 
             
      }            
                
//****************************************************************************************
//*
//*     FOUR RINGS FOUND - TARGET ZONE C
//*     
//*     This is the section of code to run when there are four rings in the stack
//*
//*
//*     lines 1116-1302
//*
//*
//*****************************************************************************************




        if (rings == 4){
        //auto plan
        //auto start!
        
        //drive 115 inches
        // driveInches(101,0.6, 0);

        
        // //places wobble goal
        // arm.setTargetPosition(675);         //  Move Arm to place wobble on mat
        // arm.setPower(.5);
        // arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // sleep(1000);                        // Give arm time to move to drop position
        // wobbleServo.setPosition(wOpen);      // Open wobble servo to release wobble
        // sleep(450);
        // driveInches(-3,0.5, 0);
        // shooter.setVelocity(2500); 
        // strafeInches(24,0.5,0);
        // driveInches(-36, 0.5,0);
        // sleep(100);
        driveTicksRelative(11700, -265, 0.6,0.25, 0 );
        arm.setTargetPosition(675);         //  Move Arm to place wobble on mat
        arm.setPower(.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(350);                        // Give arm time to move to drop position
        wobbleServo.setPosition(wOpen);      // Open wobble servo to release wobble
        sleep(150);
        shooter.setVelocity(2600);
 
 //driveTicksRelative(6977, 4649, 0.5,0.25, 0 );
 driveTicksRelative(-5200, 3000, 0.5, 0.5, 0 );

  // turn right 180 degrees
      
                 // Turn right to -180 heading
            
                 angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  //robotAngle =angles.firstAngle;
                  robotAngle= 0;
                  targetAngle = -179;
                  newTime = runtime.milliseconds();
                  while (!(targetAngle >=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = ((targetAngle -robotAngle) * 0.006) - .13;
         //         double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;
                  double left_pwr = PIDpwr * -1 ;
                  double right_pwr = PIDpwr ;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  newAngle = robotAngle;
                 if ((targetAngle < -90) && (robotAngle > 10)){
                    
                  robotAngle = (robotAngle-360);  
                      
                      
                 }
    
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  newAngle =angles.firstAngle;
                      telemetry.addData("Robot Angle", robotAngle); 
                    telemetry.addData("IMU Angle", newAngle);
                         telemetry.addData("power", PIDpwr);               
                    
                          telemetry.update();
                  }
                setPowerZero();
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

sleep(200);

//SHOOT
shootRing(3); 
shooter.setPower(0);
sleep(200);
shooter.setVelocity(2450);
intake.setPower(-1);
sleep(200);
driveInches(15,0.05,180);
//collect 
sleep(200);
shootRing(2);
driveInches(-2,0.2,180);
driveInches(10,0.1,180);
driveInches(-5,0.2,180);
sleep(200);
shootRing(2);

//driveInches(-20,0.5,180);

shooter.setPower(0);
intake.setPower(0);

          strafeInches(-3,0.7,180); 
    
    //     // Drive down to pick up next wobble
      //driveInches(20, 0.6,180);
        
    //     // Alignment code********************(sideways)
        // We need to adjust the sideways distance the robot is away from the wall 
     rightDist = rightDistance.getDistance(DistanceUnit.INCH);
      sleep(100);
      frontDist = frontDistance.getDistance(DistanceUnit.INCH);
      sleep(100);
        telemetry.addData("frontdist", frontDist);
        telemetry.addData("rightdist", rightDist);      
            telemetry.update();

      strafeDist = (rightDist - 29);
      forwardDist = (frontDist -31);
      
  //      strafeDist = strafeDist * -1;//switch polarity
            telemetry.addData("strafe", strafeDist);      
            telemetry.addData("front", forwardDist);  
            telemetry.update();   
                  strafeDist = strafeDist * .7;
    strafeInches(strafeDist, 0.6,180); 
    
        driveInches(forwardDist, 0.4,180); //***
     
    //     // pick up wobble
      sleep(750);   
        wobbleServo.setPosition(wClose);      // Close servo 
     sleep(750); 
        arm.setTargetPosition(550);         //  Move Arm to pick up wobble
        arm.setPower(.8);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);                       // Give arm time to go back to zero position
        arm.setPower(.8);
        
//   // back up to point to align to return
  
             driveInches(-84, 0.8,180); 
              
//   // turn right to get back to where the wobble drop is
   
//               // Turn right to 90 heading
            
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                if (robotAngle < 1){
                    
                    robotAngle = (-robotAngle);  
                      
                      
                  }
                  targetAngle = 90;
                  newTime = runtime.milliseconds();
                  while (!(targetAngle >=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = ((targetAngle - robotAngle) * 0.010) - .12;
         //         double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;
                  double left_pwr = PIDpwr * -1 ;
                  double right_pwr = PIDpwr ;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  newAngle = robotAngle;
                    if (robotAngle < 1){
                    
                    robotAngle = (-robotAngle);  
                      
                      
                  }
             //    oldAngle = newAngle;
              //   oldTime = newTime;
              //   newAngle = robotAngle;
              //   newTime = runtime.milliseconds();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  newAngle =angles.firstAngle;
                      telemetry.addData("Robot Angle", robotAngle); 
                    telemetry.addData("IMU Angle", newAngle); 
                          telemetry.update();
                  }
                  setPowerZero();
                  leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 
     // drive forward to drop wobble   
              driveInches(3, .4,90); 
 
    // drop wobble

    arm.setTargetPosition(675);         //  Move Arm to place wobble on mat
        arm.setPower(.8);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(200);  
          wobbleServo.setPosition(wOpen);      // Open wobble servo to release wobble
            sleep(150); 
    
    // strafe to line
     arm.setTargetPosition(550);         //  Move Arm to place wobble on mat
        arm.setPower(.8);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(200);  
            strafeInches(-35,.9, 90); 
    
 
 
 
        }
   // V TEST CODE BELOW V
 // strafeInches(25,.5,0);
    //   leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //     rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //     leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //     rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 
  //shooter.setVelocity(3000);
 //                  leftBack.setPower(.5);
//                   rightFront.setPower(0);
//                   rightBack.setPower(0); 
 // sleep(10000);
  //shooter.setPower(0);
//  shooterServo.setPosition(.43);
//  sleep(200);
//  flicker.setPosition(.3); //.43 open, .3 fire
//  sleep(200);
//  flicker.setPosition(.43);
// wobbleServo.setPosition(.5);
//  //                    leftFront.setPower(0);
 //                 leftBack.setPower(0);
//                   rightFront.setPower(0);
//                   rightBack.setPower(0); 

   //        telemetry.addData("current position:", rightFront.getCurrentPosition());
    //        telemetry.addData("Start Velocity:", startVelocity);          
 
      //      telemetry.update();
            
       //     sleep(5000);
         }    
         }

    }
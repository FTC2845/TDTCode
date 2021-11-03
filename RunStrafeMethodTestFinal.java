package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name = "StrafeTestRun ", group = "Linear Opmode")
public class RunStrafeMethodTest extends LinearOpMode  {

    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private BNO055IMU imu;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
  //  private DcMotorEx arm;
  //  private DcMotorEx shooter;
   // private Servo wobbleServo;
   // private Servo shooterServo;
   // ModernRoboticsI2cRangeSensor rightDistance;
   // ModernRoboticsI2cRangeSensor frontDistance;
 
    
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
                
    public double pwr;
    public double leftpwr=0;
    public double rightpwr=0;
    public double pfactor = (.002);

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
 
        if (ticks <= 500){
            
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
 
        if (ticks >= -500){
            
          pfactor = (power/ ticks);
                pwr = ((-remainingticks * pfactor) - .25);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher power.  For these moves, the 
 // robot has a lot of inertia, and the PID method needs to apply reverse power to apply braking versus
 // simply decreasing applied power.  To accomplish this, the PID routine measures the velocity of the 
 // robot, calculates the target velocity, then applies power to have the robot follow the velocity target.
 
 
        else {
                if (remainingticks < -500){   // When the robot is in the middle of long move, give full power
                 pwr = -power -.25;   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityState){                     // This flag is TRUE when the routine starts
                startVelocity = currentVelocity;        // Capture the current velocity at beginning of deceleration
                velocityState = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }

                targetVelocity = -(remainingticks/500)*startVelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                pwr = ((currentVelocity - targetVelocity)* -.1) -.25;   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
                if ((rightFront.getCurrentPosition() > -500) && (-power < -.3)) {
                        pfactor = (-power/ -500);
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
    
   
   
    

    @Override
    public void runOpMode() {
 

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        
    //    rightDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightDistance");
    //    frontDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontDistance");
        
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
  //      arm = hardwareMap.get(DcMotorEx.class, "arm");
   //     shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    //    wobbleServo = hardwareMap.servo.get("wobbleServo");
    //    shooterServo = hardwareMap.servo.get("shooterServo");
        
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
     //   arm.setDirection(DcMotorEx.Direction.FORWARD);
     //   shooter.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
     //   arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
     //   shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
      //  arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
        
          //wobbleServo.setPosition(0); 
        waitForStart();
        
        runtime.reset();                // Reset the main timer

        
        
         // run until the end of the match (driver presses STOP)
 
        if (opModeIsActive()) {
            
      
   // V TEST CODE BELOW V
   
         driveInches(60,0.5, 0);
        strafeInches(-2.5,.5,0);
        driveInches(-60,0.5, 0);
        strafeInches(2.5,.5,0);     
        
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
         }    // end bracket for If Op Mode Active
         }

    }

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "ArmTest", group = "Linear Opmode")
public class ArmTest extends LinearOpMode  {

    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private BNO055IMU imu;
    private DcMotorEx leftSide;
    private DcMotorEx rightSide;
    private DcMotorEx arm;
    private Servo wobbleServo;


    int positionLeft;
    int positionleftSide;

    int positionrightSide;

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
    public double leftdist;
    public double rightdist;
    public double oldTime=0;
    public double newTime=0;
    public double oldAngle=1;
    public double newAngle=0;
    public double strafeDist;

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
        final double wheelDiameter = 4;
        final double distancePerRevolution = wheelDiameter * Math.PI; //Inches
        final double ticksPerRevolution = 1400;

        return (int)(((inches) / distancePerRevolution) * ticksPerRevolution);
    }


   
//**********FORWARD AND REVERSE DRIVING**************        

    public void driveInches(double inches, double power, double targetAngle) {
        leftSide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
       


        leftSide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
       

        int ticks = inchesToTicks(inches);


        if (inches > 0) {

            while (ticks > leftSide.getCurrentPosition()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                double off_angle = robotAngle- targetAngle ;
                double remainingticks = ticks - leftSide.getCurrentPosition();
                double pwr;
                double leftpwr=0;
                double rightpwr=0;
                double pfactor = (.002);
   
                if (remainingticks> 1500){
                 pwr = power;   
                } 
                else{
                pfactor = (power / remainingticks);
                pwr = ((remainingticks * pfactor) - .05);
                }
 
                
   //     Hysteresis to prevent overshoot
              if (off_angle < 0.2 && off_angle > -0.2) {
                leftpwr = pwr;
                rightpwr = pwr;
                            telemetry.addData("flag", "OK");             
             }
            
  //    Turn robot back to heading if the current heading is off from desired heading          
                if (off_angle >= 0.2 || off_angle <= -0.2) {
                        
                        leftpwr = pwr + (off_angle * 0.1+ 0);
                        rightpwr = pwr - (off_angle * 0.1 + 0);
                        
                  telemetry.addData("flag", "correcting");              
                        
                      
 
                }    
               
         
                leftSide.setPower(leftpwr);
                rightSide.setPower(rightpwr);
              
                
                telemetry.addData("off angle", off_angle);
                telemetry.update();

                positionleftSide = leftSide.getCurrentPosition();
                positionrightSide = rightSide.getCurrentPosition();
               


 

            }
            
            leftSide.setPower(0);
            rightSide.setPower(0);
            
         

        }

     if (inches < 0) {

            while (ticks < leftSide.getCurrentPosition()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                double off_angle = robotAngle-targetAngle  ;              
                double remainingticks = ticks - leftSide.getCurrentPosition();
                double pwr;
                double leftpwr=0;
                double rightpwr=0;
                double pfactor = (.002);
                if (remainingticks> 1500){
                 pwr = power;   
                } 
                else{
                pfactor = (power / -remainingticks);
                pwr = ((remainingticks * pfactor) + .05);
                }

 //     Hysteresis to prevent overshoot
              if (off_angle < 0.2 && off_angle > -0.2) {
                leftpwr = pwr;
                rightpwr = pwr;
                            telemetry.addData("flag", "OK");             
             }
            
  //    Turn robot back to heading if the current heading is off from desired heading          
                if (off_angle >= 0.2 || off_angle <= -0.2) {
                        
                        leftpwr = pwr + (off_angle * 0.07 + 0);
                        rightpwr = pwr - (off_angle * 0.07 + 0);
                        
                  telemetry.addData("flag", "correcting");              
                        
                      
 
                }    
               
         
                leftSide.setPower(leftpwr);
                rightSide.setPower(rightpwr);
               
        
             
        telemetry.addData("off angle", off_angle);
        telemetry.update();

                positionleftSide = leftSide.getCurrentPosition();
                positionrightSide = rightSide.getCurrentPosition();
          


 

            }
            
            leftSide.setPower(0);
            rightSide.setPower(0);
          

        }
    
    }
        
    



    @Override
    public void runOpMode() {
 

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftSide = hardwareMap.get(DcMotorEx.class, "leftSide");
        
        rightSide = hardwareMap.get(DcMotorEx.class, "rightSide");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        wobbleServo = hardwareMap.servo.get("wobbleServo");

        leftSide.setDirection(DcMotorEx.Direction.REVERSE);
       
        rightSide.setDirection(DcMotorEx.Direction.FORWARD);
        
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        leftSide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
       
       
        leftSide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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

        
       
        waitForStart();
        runtime.reset();                // Reset the main timer

        
        
         // run until the end of the match (driver presses STOP)
 
        if (opModeIsActive()) {
            
            
            wobbleServo.setPosition(1);      // Lock servo hook on Wobble closed
            sleep(1000);
            arm.setTargetPosition(850);         //  Move Arm to place wobble on mat
            arm.setPower(.5);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            sleep(2000);                        // Give arm time to move to drop position

             wobbleServo.setPosition(.465);      // Open wobble servo to release wobble
             
            sleep(1000);                       // Give servo time to open
             
            arm.setTargetPosition(0);         //  Move Arm to place wobble on mat
            arm.setPower(.5);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);                       // Give arm time to go back to zero position
            arm.setPower(0);
        } 
    }
        }
    


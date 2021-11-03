package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "MainAutoBlueTwoSkystone2", group = "Linear Opmode")
public class SkystoneExampleCode extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private BNO055IMU imu;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx slider;
    private Servo draggerZero;
    private Servo draggerOne;
    private Servo gripperLeft;
    private Servo gripperRight;
    private Servo swivel;
    private Servo gripUp;
    private DcMotorEx leftLift = null;
    private DcMotorEx rightLift = null;
    private DistanceSensor frontDistance;
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;
    private DistanceSensor backDistance;
    int positionLeft;
    int positionLeftFront;
    int positionLeftBack;
    int positionRightFront;
    int positionRightBack;
    int positionRight;
    boolean stoneNotReady = true;
    boolean stoneTimeout = true;
    public int StoneTimeStart = 0;
    public int StoneIncTime = 0;
    int positionSlider;
    int LR_enc_diff;
    int RL_enc_diff;
    double drive;
    public double ycorrection = 0;
    public double rotation = 0;
    public double rotationTwo = 0;
    public double rotationOne = 0;
    public double robotAngle;
    public Orientation angles;
    public double inchYOffset;
    public double leftdist;
    public double leftdist2;
    public double leftdist3;
    public double leftdist4;
    public double leftdist5;
    public double rightdist;
    public double rightdist2;
    public double rightdist3;
    public double rightdist4;
    public double rightdist5;
    public double oldTime=0;
    public double newTime=0;
    public double oldAngle=1;
    public double newAngle=0;
    public double strafeDist;

    

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
        "AVedM9X/////AAABmepr/cyLkkrDo8488anmS8CAjidKmcpAtrUoeq+51IZ4eD8W7pJXmqk9x0LdGXS//rek1nex4IEhlcKfPf8hj1lSwWpGkbYwS/8v3Y15L1uIeBZ96H2SMin49JBdv6A/mPqjNDKxfm3wJO5mMWlE21sQz3nfobQYqmQ4o8mgAuC8EDgLBFZH8TRmvwc0lJC6ggin2MB6Tp0YZFE7kqXX1O/2K/obJ/VwQ7l2iF8VDityCRycUpUYMMhalzQlEIL1TWaN8wKBJ+LF1M5KiDe491mU848qarz129E8QvzZPAoQHKXvt5QFWsXDejBISUBXT1GRrS3Onn7DgSZpAZ8ImpQnu4rG5cls53tAvwzxT4ju";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59; // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    public int inchesToTicks(double inches) {
        final double wheelDiameter = 4;
        final double distancePerRevolution = wheelDiameter * Math.PI; //Inches
        final double ticksPerRevolution = 537.6;

        return (int)(((inches) / distancePerRevolution) * ticksPerRevolution);
    }


   
 //**********FORWARD AND REVERSE DRIVING**************        

    public void driveInches(double inches, double power, double targetAngle) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        int ticks = inchesToTicks(inches);


        if (inches > 0) {

            while (ticks > leftFront.getCurrentPosition()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                double off_angle = robotAngle- targetAngle ;
                double remainingticks = ticks - leftFront.getCurrentPosition();
                double pwr;
                double leftpwr=0;
                double rightpwr=0;
                double pfactor = (.002);
   
                if (remainingticks> 1500){
                 pwr = power;   
                } 
                else{
                pfactor = (power / remainingticks);
                pwr = ((remainingticks * pfactor) + .05);
                }
 
                
   //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
                leftpwr = pwr;
                rightpwr = pwr;
                            telemetry.addData("flag", "OK");             
             }
            
  //    Turn robot back to heading if the current heading is off from desired heading          
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        
                        leftpwr = pwr + (off_angle * 0.03 + 0);
                        rightpwr = pwr - (off_angle * 0.03 + 0);
                        
                  telemetry.addData("flag", "correcting");              
                        
                      
 
                }    
               
         
                leftFront.setPower(leftpwr);
                rightFront.setPower(rightpwr);
                leftBack.setPower(leftpwr);
                rightBack.setPower(rightpwr);
        telemetry.addData("off angle", off_angle);
        telemetry.update();

                positionLeftFront = leftFront.getCurrentPosition();
                positionRightFront = rightFront.getCurrentPosition();
                positionLeftBack = leftBack.getCurrentPosition();
                positionRightBack = rightBack.getCurrentPosition();


 

            }
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);

        }

     if (inches < 0) {

            while (ticks < leftFront.getCurrentPosition()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                double off_angle = robotAngle-targetAngle  ;              
                double remainingticks = ticks - leftFront.getCurrentPosition();
                double pwr;
                double leftpwr=0;
                double rightpwr=0;
                double pfactor = (.002);
                if (remainingticks> 1500){
                 pwr = power;   
                } 
                else{
                pfactor = (power / -remainingticks);
                pwr = ((remainingticks * pfactor) - .05);
                }

 //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
                leftpwr = pwr;
                rightpwr = pwr;
                            telemetry.addData("flag", "OK");             
             }
            
  //    Turn robot back to heading if the current heading is off from desired heading          
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        
                        leftpwr = pwr + (off_angle * 0.04 + 0);
                        rightpwr = pwr - (off_angle * 0.04 + 0);
                        
                  telemetry.addData("flag", "correcting");              
                        
                      
 
                }    
               
         
                leftFront.setPower(leftpwr);
                rightFront.setPower(rightpwr);
                leftBack.setPower(leftpwr);
                rightBack.setPower(rightpwr);
        telemetry.addData("off angle", off_angle);
        telemetry.update();

                positionLeftFront = leftFront.getCurrentPosition();
                positionRightFront = rightFront.getCurrentPosition();
                positionLeftBack = leftBack.getCurrentPosition();
                positionRightBack = rightBack.getCurrentPosition();


 

            }
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);

        }
    
    }
        
    

    //***************LEFT RIGHT STRAFING******************


    public void strafeInches(double inches, double power, double targetAngle) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int ticks = inchesToTicks(inches);
        double exponentTicks =0;
        double exponentEncoder =0; 
        double accelThresholdTicks = (ticks * .67);

        double accelCoefficient = 0;

        telemetry.addData("ticks:", ticks);

        if (inches > 0) {

            while (ticks > leftFront.getCurrentPosition()) {
 
                 angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                double off_angle = targetAngle - robotAngle;
                double remainingticks = ticks - leftFront.getCurrentPosition();
                double pwr;

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
                    
                exponentTicks = Math.pow (ticks,5);
                exponentEncoder = Math.pow (leftFront.getCurrentPosition(),5);
               double currentInchPosition = (leftFront.getCurrentPosition()* .00186);

   
                pwr=((((-power/exponentTicks)*exponentEncoder)+power)+.1);
               
                
                double r = Math.hypot(pwr, -ycorrection);
                double robotAngle = Math.atan2(-ycorrection, pwr) - Math.PI / 4;;

                double v1 = r * Math.cos(robotAngle) + rotation;
                double v2 = r * Math.sin(robotAngle) - rotation;
                double v3 = r * Math.sin(robotAngle) + rotation;
                double v4 = r * Math.cos(robotAngle) - rotation;

                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);

 

            }
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);

        }

        if (inches < 0) {

        while (ticks < leftFront.getCurrentPosition()) {
            
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                double off_angle = targetAngle - robotAngle;
                double remainingticks = ticks - leftFront.getCurrentPosition();
                double pwr;

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
                exponentTicks = Math.pow (ticks,5);
                exponentEncoder = Math.pow (leftFront.getCurrentPosition(),5);

                pwr=-((((-power/exponentTicks)*exponentEncoder)+power)+.1);

                double r = Math.hypot(pwr, -ycorrection);
                double robotAngle = Math.atan2(-ycorrection, pwr) - Math.PI / 4;;

                double v1 = r * Math.cos(robotAngle) + rotation;
                double v2 = r * Math.sin(robotAngle) - rotation;
                double v3 = r * Math.sin(robotAngle) + rotation;
                double v4 = r * Math.cos(robotAngle) - rotation;

                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);

                telemetry.addData("LeftFront", v1);
                telemetry.addData("rightFront", v2);
                telemetry.addData("leftBack", v3);
                telemetry.addData("rightBack", v4);
                telemetry.update();

            }
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);

        }

    }



    @Override
    public void runOpMode() {
 

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        draggerZero = hardwareMap.servo.get("draggerZero");
        draggerOne = hardwareMap.servo.get("draggerOne");
        gripperLeft = hardwareMap.servo.get("gripperLeft");
        gripperRight = hardwareMap.servo.get("gripperRight");
        swivel = hardwareMap.servo.get("swivel");
        gripUp = hardwareMap.servo.get("gripUp");
        
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");
        
    double frontdist = frontDistance.getDistance(DistanceUnit.MM);

    double backdist = backDistance.getDistance(DistanceUnit.INCH);
        


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
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
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
        int skystoneOrder=0;
        int targetAngle;
        double inchOffset=0;



        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List < VuforiaTrackable > allTrackables = new ArrayList < VuforiaTrackable > ();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
            .translation(0, 0, stoneZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
            .translation(-bridgeX, bridgeY, bridgeZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
            .translation(-bridgeX, bridgeY, bridgeZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
            .translation(-bridgeX, -bridgeY, bridgeZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
            .translation(bridgeX, -bridgeY, bridgeZ)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
            .translation(quadField, -halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
            .translation(-quadField, -halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
            .translation(-halfField, -quadField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
            .translation(-halfField, quadField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
            .translation(-quadField, halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
            .translation(quadField, halfField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
            .translation(halfField, quadField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
            .translation(halfField, -quadField, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch; // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch; // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0; // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable: allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

 

        // Wait for the game to start (driver presses PLAY)
        
       
        waitForStart();
        runtime.reset();                // Reset the main timer
        targetsSkyStone.activate();     // Start up the Vuforia engine
        
        
 // run until the end of the match (driver presses STOP)
 
        if (opModeIsActive()) {



// Extend the slider out to position to be able to grab skystone and lock the swivel with servo power

            slider.setTargetPosition(1300);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1);

             swivel.setPosition(.465);

 
     
// Drive robot forward 16 inches.  When aligned in centerline between the first two stones, this allows the 1st two stones to 
// be viewed in the webcam.

            driveInches(16, .4, 0);
            boolean skystoneState = false;  // this switch indicates that we have found a skystone, so set it false to start
            int skystonePosition=0;         // this variable will be used to indicate which of the three first stones is the skystone

 
            double stoneTimeStart = runtime.milliseconds();  // Get the current time before entering the skystone detect loop
            double stoneIncTime =0;
     
     //******  THis is the main detect loop for determining which of the three first stones in the row is the skystone.  Our camera
     //         will only view the first two stones.  It should take less than a second to find if either of the first two is the skystone.
     //         It could be that Stone #3 (counting left to right) is the skystone, which means that Vuforia will never find it.
     //         So we are using a timer, if after 750 milliseconds we have not detected the skystone, then we assume that #3 is the skystone.
            
            while (stoneNotReady && stoneTimeout) {
                
              

                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;
                for (VuforiaTrackable trackable: allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                    inchOffset = (translation.get(1) / mmPerInch);
                    inchYOffset = (translation.get(0) / mmPerInch);

                    if (!skystoneState){
                        
                        if (inchOffset > 1){
                            
                            skystonePosition=2;
                            skystoneState=true;
                                 
                        }
                        if (inchOffset <- 1){
                            
                            skystonePosition=1;
                            skystoneState=true;
                        }
                        
   
                    }


 

       stoneNotReady=false;
                } else {
                    telemetry.addData("Visible Target", "none");
                         rightBack.setPower(0);
                        leftFront.setPower(0);
                        rightFront.setPower(0);
                        leftBack.setPower(0);
                }
  
// This is where we check the time while in the loop.  If the loop continues for 750 milliseconds, we set the flag to false and move on

                stoneIncTime = runtime.milliseconds();
                if ((stoneIncTime- stoneTimeStart)>750){
                  stoneTimeout = false;  
                   skystonePosition = 3;
                    
                }

            }

        targetsSkyStone.deactivate();

// We drop down the rear-most arm of the gripper
     
           gripperLeft.setPosition(1);

            sleep(500);
            

// Move the slider back closer to the robot to give us more clearance in the lane

            slider.setTargetPosition(975);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1);
 
 // Drive forward a few inches to get more centered in the lane           
            
      driveInches(3, .4, 0);   
      
      
// Our previous code indicated what position the skystone was in.  This section strafes the robot to align it over the skystone

            if (skystonePosition == 3){
                
                strafeInches (12.5, .5,0);
            }
            
             if (skystonePosition == 2){
                
                strafeInches (4.5, .5,0);
            }
            
                if (skystonePosition == 1){
                
                strafeInches (-3.5, .5,0);
            }

  // Now we move forward to contact the rear of stone with the rear finger on the grabber
               
              driveInches(3, .2, 0);
              


// Close the gripper to grab the skystone
 
            gripperRight.setPosition(1);

             sleep(600);
             

 
 // Lift up the arm enough to get the skystone off the ground, but not too high so that it can go under the bridge
 
            leftLift.setTargetPosition(400);
            rightLift.setTargetPosition(400);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1);
            rightLift.setPower(1);

          
            
// Move back a bit            
            
            driveInches(-1, .4, 0);
            
//turn to the 90 degree heading

              angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  targetAngle = 89;
                  newTime = runtime.milliseconds();
                  while (!(targetAngle <=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = ((targetAngle - robotAngle) * 0.009) + .07;
         //         double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;
                  double left_pwr = PIDpwr * -1;
                  double right_pwr = PIDpwr;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  oldAngle = newAngle;
                  oldTime = newTime;
                  newAngle = robotAngle;
                  newTime = runtime.milliseconds();
                 
                  }
                  
                    leftFront.setPower(0);
                  leftBack.setPower(0);
                  rightFront.setPower(0);
                  rightBack.setPower(0); 
                  leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
            strafeInches(-1,.5,90); 

        
// Now we are facing the proper way, we need to go forward to get over to the foundation area.  Since we are starting interface
// three different spots, we need to change the amount of distance depending on the position of the skystone by putting interface
// an offset

                double ssOffset=0;
                double ssDrive =0;
                
                if (skystonePosition ==1){
                    
                    ssOffset= -8; 
                    
                }
                if (skystonePosition ==2){
                    
                    ssOffset= 0; 
                    
                }
                if (skystonePosition ==3){
                    
                    ssOffset= 8; 
                    
                }
                
// Drive forward over to the foundation, the distance includes the offset
                
 
                driveInches((78+ssOffset), .9, 90);
  

 // Now raise the arm to clear the foundation
                
            leftLift.setTargetPosition(525);
            rightLift.setTargetPosition(525);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1);
            rightLift.setPower(1);
            
            sleep(750);
            
 //turn back to the zero degree heading
            
                     
                   angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                   robotAngle =angles.firstAngle;
                  newTime = runtime.milliseconds();                   
                  targetAngle = 0;
                  while (!(targetAngle >=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = (targetAngle - robotAngle) * 0.009 - 0.07;
            //      double PIDpwr = ((targetAngle - robotAngle) * 0.035) + (((newAngle-oldAngle)/(newTime-oldTime))*2.5)+.001;
                  double left_pwr = PIDpwr * -1;
                  double right_pwr = PIDpwr;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  oldAngle = newAngle;
                  oldTime = newTime;
                  newAngle = robotAngle;
                  newTime = runtime.milliseconds();
                 
                 
                  }
                  
                    leftFront.setPower(0);
                  leftBack.setPower(0);
                  rightFront.setPower(0);
                  rightBack.setPower(0); 
                  leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
   // Get the dragger clips in a prepped position , closer to the right spot, but enough to clear the edge of foundation

            draggerOne.setPosition(.6);
            draggerZero.setPosition(.5);

// We need to adjust the sideways distance the robot is away from the wall so we can place the skystone in the center of the foundation

            leftdist = leftDistance.getDistance(DistanceUnit.INCH);
            sleep(50);
            strafeDist = (leftdist - 12.5);
            
            strafeDist = strafeDist * -1;

            strafeInches(strafeDist, .3,0); 
        
 // extend the arm to get the skystone in the 2nd row of the foundation       
  
            slider.setTargetPosition(1450);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1);         

//*****************Move forward slowly until touching foundation****************************8

            frontdist = frontDistance.getDistance(DistanceUnit.INCH);
                while (frontdist >2) {
    
                     frontdist = frontDistance.getDistance(DistanceUnit.INCH);
 
                    leftFront.setPower(.2);
                    leftBack.setPower(.2);
                    rightFront.setPower(.2);
                    rightBack.setPower(.2);
            
                }

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            


         driveInches(2, .2, 0);

            
 //*********Drop down the draggers**************
 
            draggerOne.setPosition(.8);
            draggerZero.setPosition(.3);
            
            sleep(500);
// ***** strafe sideways to pull foundation away from wall, enough so when we turn the foundation, we don't collide excessively with the wall
 
         strafeInches(15, 1,0); 
 
  //*********pull away from wall while turning, need to get foundation moved 90 degrees *************
 
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  targetAngle = 90;
                  while (!(targetAngle <=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = ((targetAngle -robotAngle  ) * 0.007) + .4;
                  double left_pwr = PIDpwr * -1;
                  double right_pwr = PIDpwr*.1;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
 //               telemetry.addData("robotAngle1: ", robotAngle );
 //                 telemetry.update();
 
                 
                  }
                  
                    leftFront.setPower(0);
                  leftBack.setPower(0);
                  rightFront.setPower(0);
                  rightBack.setPower(0); 
   
 

        
////************Strafe to depot to get it closer to the depot***********       
                  
          strafeInches(-8, 1,90); 
       
 
  
 ////************Move Forward 18 inches to get foundation closer to wall

        driveInches(16,.9,90);     
  
// Lower the arm so the skystone is closer to the foundation  
   
            leftLift.setTargetPosition(250);
            rightLift.setTargetPosition(250);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1);
            rightLift.setPower(1);
            
  
            
 // Open gripper to allow skystone to drop onto foundation
 

            gripperRight.setPosition(.56);
            

 
 // Lift up dragger clips to prepare to drive away
 
            draggerOne.setPosition(.6);
            draggerZero.setPosition(.5);
            
// Raise up the arm so we don't knock over the skystone
            
             leftLift.setTargetPosition(400);
            rightLift.setTargetPosition(400);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1);
            rightLift.setPower(1);    
            
            sleep(500);
            
// Move back 3 inches to clear foundation

            driveInches(-3,.7,90);
            
 // Check distance from the wall to get the offset value
 
            leftdist = leftDistance.getDistance(DistanceUnit.INCH);
sleep(50);
             strafeDist = (26-leftdist);


 
// strafe towards center of field to get it in the lane closest to bridge

            strafeInches(strafeDist, .6,90); 
        


  // Lower arms to get into the position we need to pick up the next skystone
      
            leftLift.setTargetPosition(0);
            rightLift.setTargetPosition(0);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1);
            rightLift.setPower(1);

// Retrace the arm to get into the right position to grab skystone
            
            slider.setTargetPosition(950);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1); 
            
// Drive back to the point where we picked up the skystone

            driveInches(-72,.8,90);
            
 // Check distance from the wall to get the offset value
 
            leftdist = leftDistance.getDistance(DistanceUnit.INCH);
sleep(50);
             strafeDist = (25.5-leftdist);


 
// strafe towards center of field to get it in the lane closest to bridge

            strafeInches(strafeDist, .3,90); 
        

// Turn robot back to the zero angle to get ready to pickup next skystone 
 
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  newTime = runtime.milliseconds();
                  targetAngle = 0;
                  while (!(targetAngle >=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                 double PIDpwr = (targetAngle - robotAngle) * 0.009 - 0.07;
            //     double PIDpwr = ((targetAngle - robotAngle) * 0.035) + (((newAngle-oldAngle)/(newTime-oldTime))*2.5)+.001;                  
                  double left_pwr = PIDpwr * -1;
                  double right_pwr = PIDpwr;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                 oldAngle = newAngle;
                  oldTime = newTime;
                  newAngle = robotAngle;
                  newTime = runtime.milliseconds();
                 
                 
                  }
                  
                    leftFront.setPower(0);
                  leftBack.setPower(0);
                  rightFront.setPower(0);
                  rightBack.setPower(0); 
                  
// Measure distance to wall and calculate where we need to be to pickup the correct skystone            
            
                rightdist = rightDistance.getDistance(DistanceUnit.INCH);
 sleep(50);
                strafeDist = (rightdist-3.5);


  
                if (skystonePosition == 3){
                
                    strafeInches (strafeDist, .5,0);
                }
            
                if (skystonePosition == 2){
                
                    strafeInches (strafeDist, .5,0);
                }
            
                if (skystonePosition == 1){
                
                    strafeInches (strafeDist - 8, .5,0);
                }

if (skystonePosition == 1 || skystonePosition == 2){

// Drive forward to contact the back edge of next skystone  
              
              driveInches(3, .15, 0);
              
 
// Close the gripper to grab the skystone
 
            gripperRight.setPosition(1);

             sleep(600);
             

 // Lift up the arm enough to get the skystone off the ground, but not too high so that it can go under the bridge
 
            leftLift.setTargetPosition(400);
            rightLift.setTargetPosition(400);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1);
            rightLift.setPower(1);

// Back up a little bit

         driveInches(-2, .3, 0);            
        
            strafeInches(-2, .5,90);
            
//turn to the 90 degree heading

              angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  targetAngle = 89;
                  newTime = runtime.milliseconds();                  
                  while (!(targetAngle <=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                double PIDpwr = ((targetAngle - robotAngle) * 0.009) + .07;
          //        double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;                
                  double left_pwr = PIDpwr * -1;
                  double right_pwr = PIDpwr;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                 oldAngle = newAngle;
                  oldTime = newTime;
                  newAngle = robotAngle;
                  newTime = runtime.milliseconds();
                  }
                  
                    leftFront.setPower(0);
                  leftBack.setPower(0);
                  rightFront.setPower(0);
                  rightBack.setPower(0); 
 
  // In case we drifted out of lane, we measure distance to the wall and move into the center of the lane           
            
            leftdist = leftDistance.getDistance(DistanceUnit.INCH);
 sleep(50);
            strafeDist = (26-leftdist);


 
// strafe towards center of field to get it in the lane 

            strafeInches(strafeDist, .5,90); 
            

        
// Now we are facing the proper way, we need to go forward to get over to the foundation area.  Since we are starting interface
// three different spots, we need to change the amount of distance depending on the position of the skystone by putting interface
// an offset

  
                
                if (skystonePosition ==1){
                    
                    ssOffset= -8; 
                    
                }
                if (skystonePosition ==2){
                    
                    ssOffset= 0; 
                    
                }
                if (skystonePosition ==3){
                    
                    ssOffset= 7; 
                    
                }
                
// Drive forward over towards the foundation, the distance includes the offset
  
  
             slider.setTargetPosition(1450);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1);         
             
 
                driveInches((70+ssOffset), .8, 90);
                
                

            leftLift.setTargetPosition(650);
            rightLift.setTargetPosition(650);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1);
            rightLift.setPower(1);
            
                                            
                                            
                                            
driveInches(20, .6, 90);
  
  

//Release Skystone
            
            gripperRight.setPosition(.56);
            
  
            
            
            sleep(500);
            
// Drive back to underneath bridge            
            
                driveInches(-5, .5, 90);
    
    // Drop the arm to bottom position
  
  
          leftLift.setTargetPosition(0);
            rightLift.setTargetPosition(0);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(.4);
            rightLift.setPower(.4);
            
     driveInches(-35, .5, 90);
     
}
  
 else {
     
     
             swivel.setPosition(.58);
              
              
                   angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                   robotAngle =angles.firstAngle;
                  newTime = runtime.milliseconds();                   
                  targetAngle = -19;
                  while (!(targetAngle >=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = (targetAngle - robotAngle) * 0.015 - 0.07;
            //      double PIDpwr = ((targetAngle - robotAngle) * 0.035) + (((newAngle-oldAngle)/(newTime-oldTime))*2.5)+.001;
                  double left_pwr = PIDpwr * -1;
                  double right_pwr = PIDpwr;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  oldAngle = newAngle;
                  oldTime = newTime;
                  newAngle = robotAngle;
                  newTime = runtime.milliseconds();
                 
                 
                  }
                  
                    leftFront.setPower(0);
                  leftBack.setPower(0);
                  rightFront.setPower(0);
                  rightBack.setPower(0); 

            
              slider.setTargetPosition(1450);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1);  
                       sleep(500);  
 
// Close the gripper to grab the skystone
 
            gripperRight.setPosition(1);

             sleep(600);
             
            swivel.setPosition(.465);
                      
                      
                   slider.setTargetPosition(1000);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1);
                sleep(250);         

 // Lift up the arm enough to get the skystone off the ground, but not too high so that it can go under the bridge
 
            leftLift.setTargetPosition(400);
            rightLift.setTargetPosition(400);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(1);
            rightLift.setPower(1);
            
            strafeInches (-2, .3,-25);

                sleep(250); 

            
            
//turn to the -90 degree heading

              angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  targetAngle = 89;
                  newTime = runtime.milliseconds();
                  while (!(targetAngle <=robotAngle  )) { //while target angle is NOT less than equal to bot angle, if negative switch it :)
                  double PIDpwr = ((targetAngle - robotAngle) * 0.009) + .07;
         //         double PIDpwr = ((targetAngle - robotAngle) * 0.035) - (((newAngle-oldAngle)/(newTime-oldTime))*2.5)-.001;
                  double left_pwr = PIDpwr * -1;
                  double right_pwr = PIDpwr;
                  leftFront.setPower(left_pwr);
                  leftBack.setPower(left_pwr);
                  rightFront.setPower(right_pwr);
                  rightBack.setPower(right_pwr);
                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                  robotAngle =angles.firstAngle;
                  oldAngle = newAngle;
                  oldTime = newTime;
                  newAngle = robotAngle;
                  newTime = runtime.milliseconds();
                 
                  }
                  
                    leftFront.setPower(0);
                  leftBack.setPower(0);
                  rightFront.setPower(0);
                  rightBack.setPower(0); 
                  leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
                  
                  
             
 

                  
 

        
// Now we are facing the proper way, we need to go forward to get over to the foundation area.  Since we are starting interface
// three different spots, we need to change the amount of distance depending on the position of the skystone by putting interface
// an offset

  
                
                if (skystonePosition ==1){
                    
                    ssOffset= -8; 
                    
                }
                if (skystonePosition ==2){
                    
                    ssOffset= 0; 
                    
                }
                if (skystonePosition ==3){
                    
                    ssOffset= 10; 
                    
                }
                
// Drive forward over towards the foundation, the distance includes the offset
  
  
             slider.setTargetPosition(1450);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1);         
             
 
                driveInches((65+ssOffset), .8, 90);
                

//Release Skystone
            
            gripperRight.setPosition(.56);
            
  
    
    // Drop the arm to bottom position
  
  
          leftLift.setTargetPosition(0);
            rightLift.setTargetPosition(0);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setPower(.4);
            rightLift.setPower(.4);
            
     driveInches(-25, .5, 90);
     
 }
 
  
        
        }    
        }
    }

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Autonomous(name="Encoder Test Bot2", group="Linear Opmode")

public class EncoderTestBot2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
   private DcMotor leftFront;
    private DcMotor rightFront;
      private DcMotor leftBack;
    private DcMotor rightBack;
   
    int positionLeftFront;
    int positionLeftBack;
    int positionRightFront;
    int positionRightBack;    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


         leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
       
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack"); 
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
         leftFront.setDirection(DcMotor.Direction.REVERSE);
         leftBack.setDirection(DcMotor.Direction.REVERSE);       
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



 
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


        positionLeftFront = leftFront.getCurrentPosition();
        positionRightFront = rightFront.getCurrentPosition();
        positionLeftBack = leftBack.getCurrentPosition();
        positionRightBack = rightBack.getCurrentPosition();
      
 
        telemetry.addData("leftFront Encoder:", positionLeftFront);
        telemetry.addData("rightFront Encoder:", positionRightFront);
        telemetry.addData("leftBack Encoder:", positionLeftBack);
        telemetry.addData("rightBack Encoder:", positionRightBack);
       

            telemetry.update();
            
         
 
        

    


        }
    }
}

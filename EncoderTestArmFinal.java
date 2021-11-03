

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Autonomous(name="Encoder Test Arm", group="Linear Opmode")

public class EncoderTestArm extends LinearOpMode {

   // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//   private DcMotor leftSide;
//     private DcMotor rightSide;
    private DcMotorEx arm;  
   
    // int positionLeftSide;

    // int positionRightSide;
    
    int positionarm;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // leftSide = hardwareMap.get(DcMotor.class,"leftSide");
       
        // rightSide = hardwareMap.get(DcMotor.class,"rightSide");
        
        arm = hardwareMap.get(DcMotorEx.class, "arm"); 
 
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //  leftSide.setDirection(DcMotor.Direction.REVERSE);
       
        // rightSide.setDirection(DcMotor.Direction.FORWARD);
        
        arm.setDirection(DcMotor.Direction.REVERSE);
        
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
        // leftSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rightSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


        // positionLeftSide = leftSide.getCurrentPosition();
        // positionRightSide = rightSide.getCurrentPosition();
        positionarm = arm.getCurrentPosition();
      
 
   //     telemetry.addData("leftSide Encoder:", positionLeftSide);
   //     telemetry.addData("rightSide Encoder:", positionRightSide);
        telemetry.addData("arm:", positionarm);

            telemetry.update();
            
         
 
        

    


        }
    }
}

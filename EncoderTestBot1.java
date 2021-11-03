

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Autonomous(name="Encoder Test Bot1", group="Linear Opmode")

public class EncoderTestBot1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
   private DcMotor rightBack;
    private DcMotor rightFront;
   
    int positionrightBack;

    int positionRightFront;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


          rightBack = hardwareMap.get(DcMotor.class,"rightBack");
       
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
 
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
         rightBack.setDirection(DcMotor.Direction.FORWARD);
       
        rightFront.setDirection(DcMotor.Direction.FORWARD);
      
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


        positionrightBack = rightBack.getCurrentPosition();
        positionRightFront = rightFront.getCurrentPosition();
      
 
        telemetry.addData("rightBack Encoder:", positionrightBack);
        telemetry.addData("rightFront Encoder:", positionRightFront);
       

            telemetry.update();
            
         
 
        

    


        }
    }
}

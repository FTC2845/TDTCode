package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="IMUTEST", group="Iterative Opmode")

public class IMUTest extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    boolean doTurn = false;
    int targetAngle = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        
        leftFront = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightFront = hardwareMap.dcMotor.get("right_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters imuparams = new BNO055IMU.Parameters();
        imuparams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparams.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;
        imuparams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        
        imu.initialize(imuparams);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Orientation robotAngle = imu.getAngularOrientation();
        
        if (doTurn) {
            rightFront.setPower(-0.3);
            rightBack.setPower(-0.3);
            leftBack.setPower(0.3);
            leftFront.setPower(0.3);
            
            float offset = Math.abs(robotAngle.firstAngle) - targetAngle;
            
            if (robotAngle.firstAngle < 0) {
                offset *= -1;
            }
            
            if (offset <= 5 && offset >= -5)
            {
                doTurn = false;
                runtime.reset();
            }
        } else {
            float offset = Math.abs(robotAngle.firstAngle) - targetAngle;
            
            if (robotAngle.firstAngle < 0) {
                offset *= -1;
            }
            
            if (offset < 0){ //z
                rightFront.setPower(0.8);
                rightBack.setPower(0.8);
                leftBack.setPower(1);
                leftFront.setPower(1);
            }
                
            else if(offset > 0){
                rightFront.setPower(1);
                rightBack.setPower(1);
                leftBack.setPower(0.8);
                leftFront.setPower(0.8);
            }
            else{
                rightFront.setPower(1);
                rightBack.setPower(1);
                leftBack.setPower(1);
                leftFront.setPower(1);
            }
            
            if (runtime.time() > 10) {
                doTurn = true;
                if (targetAngle == 0) {
                    targetAngle = 180;
                } else {
                    targetAngle = 0;
                }
            }
        }
            
        telemetry.addData("Angle", imu.getAngularOrientation());
        telemetry.addData("Acceleration", imu.getAcceleration());
        telemetry.addData("Position", imu.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

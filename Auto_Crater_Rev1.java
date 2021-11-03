package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

@Autonomous(name = "Auto_Crater_Rev1 (Blocks to Java)", group = "")
public class Auto_Crater_Rev1 extends LinearOpMode {

  private BNO055IMU imu;
  private DcMotor left_front;
  private DcMotor left_back;
  private DigitalChannel lander_lift_touch_top;
  private VuforiaRoverRuckus vuforiaRoverRuckus;
  private TfodRoverRuckus tfodRoverRuckus;
  private DcMotor right_front;
  private DcMotor right_back;
  private DcMotor arm_lift;
  private DcMotor lander_lift;
  private Servo lander_hook;
  private DistanceSensor left_dist;
  private DcMotor mineralgather;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    List<Recognition> recognitions;
    double goldMineralX;
    double GoldState;
    BNO055IMU.Parameters imuParameters;
    Orientation angles;
    double bot_angle;
    double start_dist;
    double mtrpwr;
    double target_angle;
    double PIDpwr;
    double left_pwr;
    double right_pwr;
    double end_dist;
    double off_angle;
    double incremental_dist;
    double time2;

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    left_front = hardwareMap.dcMotor.get("left_front");
    left_back = hardwareMap.dcMotor.get("left_back");
    lander_lift_touch_top = hardwareMap.digitalChannel.get("lander_lift_touch_top");
    vuforiaRoverRuckus = new VuforiaRoverRuckus();
    tfodRoverRuckus = new TfodRoverRuckus();
    right_front = hardwareMap.dcMotor.get("right_front");
    right_back = hardwareMap.dcMotor.get("right_back");
    arm_lift = hardwareMap.dcMotor.get("arm_lift");
    lander_lift = hardwareMap.dcMotor.get("lander_lift");
    lander_hook = hardwareMap.servo.get("lander_hook");
    left_dist = hardwareMap.get(DistanceSensor.class, "left_dist");
    mineralgather = hardwareMap.dcMotor.get("mineralgather");

    // Put initialization blocks here.
    imuParameters = new BNO055IMU.Parameters();
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imuParameters.loggingEnabled = false;
    imu.initialize(imuParameters);
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    bot_angle = angles.firstAngle;
    GoldState = 0;
    time2 = 0;
    left_front.setDirection(DcMotorSimple.Direction.REVERSE);
    left_back.setDirection(DcMotorSimple.Direction.REVERSE);
    lander_lift_touch_top.setMode(DigitalChannel.Mode.INPUT);
    vuforiaRoverRuckus.initialize(
        "", // vuforiaLicenseKey
        VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
        true, // useExtendedTracking
        false, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        0, // yAngle
        0, // zAngle
        true); // useCompetitionFieldTargetLocations
    tfodRoverRuckus.initialize(vuforiaRoverRuckus, 0.4F, true, false);
    left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    telemetry.addData(">", "Ready to Press Start");
    telemetry.update();
    waitForStart();
    lander_lift.setPower(1);
    while (lander_lift_touch_top.getState()) {
    }
    lander_lift.setPower(-1);
    sleep(20);
    lander_lift.setPower(0);
    lander_hook.setPosition(0);
    sleep(800);
    arm_lift.setTargetPosition(1400);
    arm_lift.setPower(0.6);
    while (!!arm_lift.isBusy()) {
    }
    arm_lift.setPower(0.2);
    tfodRoverRuckus.activate();
    while (!(GoldState != 0 || isStopRequested())) {
      // Put loop blocks here.
      recognitions = tfodRoverRuckus.getRecognitions();
      goldMineralX = -1;
      for (Recognition recognition : recognitions) {
        if (recognition.getLabel().equals("Gold Mineral")) {
          goldMineralX = recognition.getLeft();
        }
      }
      if (goldMineralX > 0 && goldMineralX < 400) {
        telemetry.addData("Gold Mineral Position", "Left");
        GoldState = 1;
      }
      if (goldMineralX > 400 && goldMineralX < 700) {
        telemetry.addData("Gold Mineral Position", "Center");
        GoldState = 2;
      }
      if (goldMineralX > 700) {
        telemetry.addData("Gold Mineral Position", "Right");
        GoldState = 3;
      }
      telemetry.addData("Gold State", GoldState);
      telemetry.update();
    }
    tfodRoverRuckus.deactivate();
    arm_lift.setTargetPosition(0);
    arm_lift.setPower(1);
    while (!!arm_lift.isBusy()) {
    }
    arm_lift.setPower(0);
    left_front.setTargetPosition(950);
    left_back.setTargetPosition(950);
    right_front.setTargetPosition(950);
    right_back.setTargetPosition(950);
    mtrpwr = 0.6;
    left_front.setPower(mtrpwr);
    right_front.setPower(mtrpwr);
    left_back.setPower(mtrpwr);
    right_back.setPower(mtrpwr);
    while (!!left_front.isBusy()) {
    }
    left_front.setPower(0);
    right_front.setPower(0);
    left_back.setPower(0);
    right_back.setPower(0);
    left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    if (GoldState == 2) {
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(2200);
      left_back.setTargetPosition(2200);
      right_front.setTargetPosition(2200);
      right_back.setTargetPosition(2200);
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setTargetPosition(600);
      left_back.setTargetPosition(600);
      right_front.setTargetPosition(600);
      right_back.setTargetPosition(600);
      mtrpwr = -0.6;
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      target_angle = 90;
      while (!(target_angle <= bot_angle)) {
        PIDpwr = (target_angle - bot_angle) * 0.01 + 0.04;
        left_pwr = PIDpwr * -1;
        right_pwr = PIDpwr;
        left_front.setPower(left_pwr);
        left_back.setPower(left_pwr);
        right_front.setPower(right_pwr);
        right_back.setPower(right_pwr);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(5500);
      left_back.setTargetPosition(5500);
      right_front.setTargetPosition(5500);
      right_back.setTargetPosition(5500);
      mtrpwr = 0.6;
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!right_front.isBusy()) {
      }
      while (!(mtrpwr <= 0)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr - 0.1;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      target_angle = 135;
      while (!(target_angle <= bot_angle)) {
        PIDpwr = (target_angle - bot_angle) * 0.01 + 0.04;
        left_pwr = PIDpwr * -1;
        right_pwr = PIDpwr;
        left_front.setPower(left_pwr);
        left_back.setPower(left_pwr);
        right_front.setPower(right_pwr);
        right_back.setPower(right_pwr);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      start_dist = left_dist.getDistance(DistanceUnit.MM);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(1440);
      left_back.setTargetPosition(1440);
      right_front.setTargetPosition(1440);
      right_back.setTargetPosition(1440);
      mtrpwr = 0.3;
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      end_dist = left_dist.getDistance(DistanceUnit.MM);
      incremental_dist = start_dist - end_dist;
      off_angle = Math.atan(incremental_dist / 318) / Math.PI * 180;
      target_angle = off_angle + bot_angle;
      telemetry.addData("start", start_dist);
      telemetry.addData("enddist", end_dist);
      telemetry.addData("incdist", incremental_dist);
      telemetry.addData("offangle", off_angle);
      telemetry.addData("target", target_angle);
      telemetry.addData("botangle", bot_angle);
      telemetry.update();
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      if (target_angle < bot_angle) {
        while (!(target_angle >= bot_angle)) {
          PIDpwr = (target_angle - bot_angle) * 0.015 - 0.04;
          left_pwr = PIDpwr * -1;
          right_pwr = PIDpwr;
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          bot_angle = angles.firstAngle;
        }
      } else {
      }
      if (target_angle > bot_angle) {
        while (!(target_angle <= bot_angle)) {
          PIDpwr = (bot_angle - target_angle) * -0.015 + 0.04;
          left_pwr = PIDpwr * -1;
          right_pwr = PIDpwr;
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          bot_angle = angles.firstAngle;
        }
      } else {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(3200);
      left_back.setTargetPosition(3200);
      right_front.setTargetPosition(3200);
      right_back.setTargetPosition(3200);
      mtrpwr = 0.6;
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      while (!(mtrpwr <= 0)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr - 0.1;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      mineralgather.setPower(1);
      sleep(200);
      mineralgather.setPower(0);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setTargetPosition(-8000);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(-10000);
      left_back.setTargetPosition(-10000);
      right_front.setTargetPosition(-10000);
      right_back.setTargetPosition(-10000);
      left_pwr = -0.9;
      right_pwr = -0.9;
      left_front.setPower(left_pwr);
      left_back.setPower(left_pwr);
      right_front.setPower(right_pwr);
      right_back.setPower(right_pwr);
      while (!!right_front.isBusy()) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
        off_angle = bot_angle - target_angle;
        // Hysteresis to prevent overshoot
        if (off_angle < 0.5 || off_angle > -0.5) {
          left_pwr = -0.8;
          right_pwr = -0.9;
        }
        if (off_angle >= 0.5 || off_angle <= -0.5) {
          if (bot_angle > target_angle) {
            left_pwr = left_pwr + off_angle * 0.07 + 0;
          }
          if (bot_angle < target_angle) {
            left_pwr = left_pwr + off_angle * 0.05 + 0;
          }
          if (left_pwr < -1) {
            left_pwr = -1;
          }
          if (right_pwr < -1) {
            right_pwr = -1;
          }
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
        }
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
    }
    if (GoldState == 1) {
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      target_angle = 50;
      while (!(target_angle <= bot_angle)) {
        PIDpwr = (target_angle - bot_angle) * 0.01 + 0.04;
        left_pwr = PIDpwr * -1;
        right_pwr = PIDpwr;
        left_front.setPower(left_pwr);
        left_back.setPower(left_pwr);
        right_front.setPower(right_pwr);
        right_back.setPower(right_pwr);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(4400);
      left_back.setTargetPosition(4400);
      right_front.setTargetPosition(4400);
      right_back.setTargetPosition(4400);
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      target_angle = 135;
      while (!(target_angle <= bot_angle)) {
        PIDpwr = (target_angle - bot_angle) * 0.0075 + 0.04;
        left_pwr = PIDpwr * -1;
        right_pwr = PIDpwr;
        left_front.setPower(left_pwr);
        left_back.setPower(left_pwr);
        right_front.setPower(right_pwr);
        right_back.setPower(right_pwr);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      start_dist = left_dist.getDistance(DistanceUnit.MM);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(1440);
      left_back.setTargetPosition(1440);
      right_front.setTargetPosition(1440);
      right_back.setTargetPosition(1440);
      mtrpwr = 0.3;
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      end_dist = left_dist.getDistance(DistanceUnit.MM);
      incremental_dist = start_dist - end_dist;
      off_angle = Math.atan(incremental_dist / 318) / Math.PI * 180;
      target_angle = off_angle + bot_angle;
      telemetry.addData("start", start_dist);
      telemetry.addData("enddist", end_dist);
      telemetry.addData("incdist", incremental_dist);
      telemetry.addData("offangle", off_angle);
      telemetry.addData("target", target_angle);
      telemetry.addData("botangle", bot_angle);
      telemetry.update();
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      if (target_angle < bot_angle) {
        while (!(target_angle >= bot_angle)) {
          PIDpwr = (target_angle - bot_angle) * 0.015 - 0.04;
          left_pwr = PIDpwr * -1;
          right_pwr = PIDpwr;
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          bot_angle = angles.firstAngle;
        }
      } else {
      }
      if (target_angle > bot_angle) {
        while (!(target_angle <= bot_angle)) {
          PIDpwr = (bot_angle - target_angle) * -0.015 + 0.04;
          left_pwr = PIDpwr * -1;
          right_pwr = PIDpwr;
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          bot_angle = angles.firstAngle;
        }
      } else {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(6500);
      left_back.setTargetPosition(6500);
      right_front.setTargetPosition(6500);
      right_back.setTargetPosition(6500);
      mtrpwr = 0.1;
      while (!(mtrpwr >= 0.6)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr + 0.1;
      }
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!right_front.isBusy()) {
      }
      while (!(mtrpwr <= 0)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr - 0.05;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      mineralgather.setPower(1);
      sleep(300);
      mineralgather.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(-8500);
      left_back.setTargetPosition(-8500);
      right_front.setTargetPosition(-8500);
      right_back.setTargetPosition(-8500);
      mtrpwr = -0.1;
      while (!(mtrpwr <= -0.85)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr - 0.05;
      }
      left_pwr = -0.9;
      right_pwr = -0.9;
      left_front.setPower(left_pwr);
      left_back.setPower(left_pwr);
      right_front.setPower(right_pwr);
      right_back.setPower(right_pwr);
      while (!!right_front.isBusy()) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
        off_angle = bot_angle - target_angle;
        if (off_angle < 0.5 || off_angle > -0.5) {
          left_pwr = -0.8;
          right_pwr = -0.9;
        }
        if (off_angle >= 0.5 || off_angle <= -0.5) {
          if (bot_angle > target_angle) {
            left_pwr = left_pwr + off_angle * 0.07 + 0;
          }
          if (bot_angle < target_angle) {
            left_pwr = left_pwr + off_angle * 0.05 + 0;
          }
          if (left_pwr < -1) {
            left_pwr = -1;
          }
          if (right_pwr < -1) {
            right_pwr = -1;
          }
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
        }
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
    }
    if (GoldState == 3) {
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      target_angle = -45;
      while (!(target_angle >= bot_angle)) {
        PIDpwr = (bot_angle - target_angle) * -0.01 - 0.04;
        left_pwr = PIDpwr * -1;
        right_pwr = PIDpwr;
        left_front.setPower(left_pwr);
        left_back.setPower(left_pwr);
        right_front.setPower(right_pwr);
        right_back.setPower(right_pwr);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(2700);
      left_back.setTargetPosition(2700);
      right_front.setTargetPosition(2700);
      right_back.setTargetPosition(2700);
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setTargetPosition(1000);
      left_back.setTargetPosition(1000);
      right_front.setTargetPosition(1000);
      right_back.setTargetPosition(1000);
      mtrpwr = -0.6;
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      target_angle = 90;
      while (!(target_angle <= bot_angle)) {
        if (bot_angle < 0) {
          bot_angle = bot_angle * -1;
        }
        PIDpwr = (target_angle - bot_angle) * 0.008 + 0.04;
        left_pwr = PIDpwr * -1;
        right_pwr = PIDpwr;
        left_front.setPower(left_pwr);
        left_back.setPower(left_pwr);
        right_front.setPower(right_pwr);
        right_back.setPower(right_pwr);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(5800);
      left_back.setTargetPosition(5800);
      right_front.setTargetPosition(5800);
      right_back.setTargetPosition(5800);
      mtrpwr = 0.8;
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!right_front.isBusy()) {
      }
      while (!(mtrpwr <= 0)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr - 0.1;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      target_angle = 135;
      while (!(target_angle <= bot_angle)) {
        PIDpwr = (target_angle - bot_angle) * 0.0075 + 0.04;
        left_pwr = PIDpwr * -1;
        right_pwr = PIDpwr;
        left_front.setPower(left_pwr);
        left_back.setPower(left_pwr);
        right_front.setPower(right_pwr);
        right_back.setPower(right_pwr);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      start_dist = left_dist.getDistance(DistanceUnit.MM);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(1440);
      left_back.setTargetPosition(1440);
      right_front.setTargetPosition(1440);
      right_back.setTargetPosition(1440);
      mtrpwr = 0.3;
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!left_front.isBusy()) {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      end_dist = left_dist.getDistance(DistanceUnit.MM);
      incremental_dist = start_dist - end_dist;
      off_angle = Math.atan(incremental_dist / 318) / Math.PI * 180;
      target_angle = off_angle + bot_angle;
      telemetry.addData("start", start_dist);
      telemetry.addData("enddist", end_dist);
      telemetry.addData("incdist", incremental_dist);
      telemetry.addData("offangle", off_angle);
      telemetry.addData("target", target_angle);
      telemetry.addData("botangle", bot_angle);
      telemetry.update();
      left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      if (target_angle < bot_angle) {
        while (!(target_angle >= bot_angle)) {
          PIDpwr = (target_angle - bot_angle) * 0.015 - 0.04;
          left_pwr = PIDpwr * -1;
          right_pwr = PIDpwr;
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          bot_angle = angles.firstAngle;
        }
      } else {
      }
      if (target_angle > bot_angle) {
        while (!(target_angle <= bot_angle)) {
          PIDpwr = (bot_angle - target_angle) * -0.015 + 0.04;
          left_pwr = PIDpwr * -1;
          right_pwr = PIDpwr;
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          bot_angle = angles.firstAngle;
        }
      } else {
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(2700);
      left_back.setTargetPosition(2700);
      right_front.setTargetPosition(2700);
      right_back.setTargetPosition(2700);
      mtrpwr = 0.1;
      while (!(mtrpwr >= 0.6)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr + 0.1;
      }
      left_front.setPower(mtrpwr);
      left_back.setPower(mtrpwr);
      right_front.setPower(mtrpwr);
      right_back.setPower(mtrpwr);
      while (!!right_front.isBusy()) {
      }
      while (!(mtrpwr <= 0)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr - 0.05;
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
      mineralgather.setPower(1);
      sleep(300);
      mineralgather.setPower(0);
      left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_front.setTargetPosition(-8000);
      left_back.setTargetPosition(-8000);
      right_front.setTargetPosition(-8000);
      right_back.setTargetPosition(-8000);
      mtrpwr = -0.1;
      while (!(mtrpwr <= -0.85)) {
        left_front.setPower(mtrpwr);
        left_back.setPower(mtrpwr);
        right_front.setPower(mtrpwr);
        right_back.setPower(mtrpwr);
        mtrpwr = mtrpwr - 0.05;
      }
      left_pwr = -0.9;
      right_pwr = -0.9;
      left_front.setPower(left_pwr);
      left_back.setPower(left_pwr);
      right_front.setPower(right_pwr);
      right_back.setPower(right_pwr);
      while (!!right_front.isBusy()) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        bot_angle = angles.firstAngle;
        off_angle = bot_angle - target_angle;
        if (off_angle < 0.5 || off_angle > -0.5) {
          left_pwr = -0.8;
          right_pwr = -0.9;
        }
        if (off_angle >= 0.5 || off_angle <= -0.5) {
          if (bot_angle > target_angle) {
            left_pwr = left_pwr + off_angle * 0.07 + 0;
          }
          if (bot_angle < target_angle) {
            left_pwr = left_pwr + off_angle * 0.05 + 0;
          }
          if (left_pwr < -1) {
            left_pwr = -1;
          }
          if (right_pwr < -1) {
            right_pwr = -1;
          }
          left_front.setPower(left_pwr);
          left_back.setPower(left_pwr);
          right_front.setPower(right_pwr);
          right_back.setPower(right_pwr);
        }
      }
      left_front.setPower(0);
      left_back.setPower(0);
      right_front.setPower(0);
      right_back.setPower(0);
    }

    vuforiaRoverRuckus.close();
    tfodRoverRuckus.close();
  }
}

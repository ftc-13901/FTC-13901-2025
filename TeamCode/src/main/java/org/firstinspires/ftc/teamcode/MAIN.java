package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "MAIN")
@Disabled
public class MAIN extends LinearOpMode {

  private DcMotor flywheel;
  private DcMotor coreHex;
  private DcMotor backLeft;
  private DcMotor frontLeft;
  private CRServo servo;
  private VoltageSensor ControlHub_VoltageSensor;
  private DcMotor frontRight;
  private DcMotor backRight;
  private ServoImplEx righty;
  private IMU imu;
  private ServoImplEx lefty;

  double speedScale;
  int ShotVelocity;
  int farVelocity;
  double slowSpeed;
  int frontServoPosition;
  int shot_tolerance;
  ElapsedTime ServosIdleTimer;
  double FullSpeedScale;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    int maxVelocity;

    flywheel = hardwareMap.get(DcMotor.class, "flywheel");
    coreHex = hardwareMap.get(DcMotor.class, "coreHex");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    servo = hardwareMap.get(CRServo.class, "servo");
    ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    righty = hardwareMap.get(ServoImplEx.class, "righty");
    imu = hardwareMap.get(IMU.class, "imu");
    lefty = hardwareMap.get(ServoImplEx.class, "lefty");

    // Setting the direction and mode for the motors
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    flywheel.setDirection(DcMotor.Direction.REVERSE);
    coreHex.setDirection(DcMotor.Direction.FORWARD);
    backLeft.setDirection(DcMotor.Direction.REVERSE);
    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    servo.setPower(0);
    // Setting our velocity targets. These values are in ticks per second!
    ShotVelocity = 1300;
    farVelocity = 1500;
    maxVelocity = 2200;
    slowSpeed = 0.25;
    shot_tolerance = 80;
    ((DcMotorEx) flywheel).setVelocityPIDFCoefficients(1.0204, 0, 0.00035, 14.5);
    set_mechanum_config();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        if (ControlHub_VoltageSensor.getVoltage() > 7) {
          // Calling our functions while the OpMode is running
          setFlywheelVelocity();
          manualCoreHexAndServoControl();
          if (!gamepad1.ps) {
            mechanum_junk();
          } else {
            drive(0, 0, 0);
            coreHex.setPower(-1);
            ((DcMotorEx) flywheel).setVelocity(-3000);
            servo.setPower(1);
          }
          controlFrontServos();
          telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
          telemetry.addData("feeder", coreHex.getPower());
          telemetry.addData("battery", ControlHub_VoltageSensor.getVoltage());
          telemetry.addData("flywheel target velocity", ShotVelocity);
          telemetry.addData("shot tolorance", shot_tolerance);
          telemetry.update();
        } else {
          telemetry.addLine("battery low");
          telemetry.update();
        }
      }
    }
  }

  /**
   * Describe this function...
   */
  private void set_mechanum_config() {
    DcMotor.ZeroPowerBehavior driveZeroPowerBehavior;

    // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder wires, you should remove these
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    righty.setDirection(Servo.Direction.REVERSE);
    driveZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    frontLeft.setZeroPowerBehavior(driveZeroPowerBehavior);
    frontRight.setZeroPowerBehavior(driveZeroPowerBehavior);
    backLeft.setZeroPowerBehavior(driveZeroPowerBehavior);
    backRight.setZeroPowerBehavior(driveZeroPowerBehavior);
    slowSpeed = 0.5;
    FullSpeedScale = 0.7;
    // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics Control
    // Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction
    // that the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    ServosIdleTimer = new ElapsedTime();
    waitForStart();
    // sets the front servos position to 0 - nik
    lefty.setPosition(0);
    righty.setPosition(0);
    frontServoPosition = 0;
  }

  /**
   * Describe this function...
   */
  private void mechanum_junk() {
    if (gamepad2.a) {
      imu.resetYaw();
    }
    speedScale = Math.max(1 - gamepad1.right_trigger, slowSpeed);
    telemetry.addData("speedScale", speedScale);
    telemetry.addData("forward", gamepad1.left_stick_y * speedScale);
    if (!gamepad2.right_bumper) {
      drive((int) (gamepad1.left_stick_y * speedScale), (int) -(gamepad1.left_stick_x * speedScale), (int) -(gamepad1.right_stick_x * speedScale));
    } else {
      drive_relative(gamepad1.left_stick_y * speedScale, -(gamepad1.left_stick_x * speedScale), (int) -(gamepad1.right_stick_x * speedScale));
    }
  }

  /**
   * Describe this function...
   */
  private void controlFrontServos() {
    telemetry.addData("leftyPos", lefty.getPosition());
    telemetry.addData("rightyPos", righty.getPosition());
    if (gamepad2.back || gamepad2.start) {
      lefty.setPwmEnable();
      righty.setPwmEnable();
      ServosIdleTimer.reset();
    }
    if (frontServoPosition <= 1 && gamepad2.start) {
      frontServoPosition = (int) (frontServoPosition + 0.05);
    } else if (frontServoPosition >= 0 && gamepad2.back) {
      frontServoPosition = (int) (frontServoPosition - 0.05);
    }
    if (2000 < ServosIdleTimer.milliseconds()) {
      lefty.setPwmDisable();
      righty.setPwmDisable();
    } else {
      lefty.setPosition(frontServoPosition);
      righty.setPosition(frontServoPosition);
    }
  }

  /**
   * Describe this function...
   */
  private void drive(int forward, int right, int rotate) {
    double maxPower;
    int frontLeftPower;
    int frontRightPower;
    int backRightPower;
    int backLeftPower;
    double maxSpeed;

    maxPower = 0;
    frontLeftPower = forward + right + rotate;
    frontRightPower = forward - (right + rotate);
    backRightPower = forward + (right - rotate);
    backLeftPower = forward - (right - rotate);
    // This is needed to make sure we don't pass > 1.0 to any wheel.
    // It allows us to keep all of the motors in proportion to what they should
    // be and not get clipped
    maxPower = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower), maxPower));
    if (maxPower <= FullSpeedScale * 0.9) {
      maxPower = 1 * 1;
    }
    // Change maxSpeed to be less than 1 for outreaches.  Do NOT change to be greater than 1
    maxSpeed = speedScale * FullSpeedScale;
    frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
    frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
    backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
    backRight.setPower(maxSpeed * (backRightPower / maxPower));
  }

  /**
   * Describe this function...
   */
  private void drive_relative(double forward_relative, double right_relative, int rotate_relative) {
    double theta;
    double r;

    theta = Math.atan2(forward_relative, right_relative) / Math.PI * 180;
    r = Math.sqrt(forward_relative * forward_relative + right_relative * right_relative);
    theta = AngleUnit.DEGREES.normalize(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    drive((int) (r * Math.sin(theta / 180 * Math.PI)), (int) (r * Math.cos(theta / 180 * Math.PI)), rotate_relative);
  }

  /**
   * Describe this function...
   */
  private void manualCoreHexAndServoControl() {
    // Manual control for the Core Hex feeder
    // Manual control for the hopper's servo
    if (gamepad1.y) {
      coreHex.setPower(-0.5);
    }
  }

  /**
   * Describe this function...
   */
  private void bankShotAuto() {
    ((DcMotorEx) flywheel).setVelocity(ShotVelocity);
    coreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    servo.setPower(1);
    if (shot_tolerance > Math.abs(((DcMotorEx) flywheel).getVelocity() - ShotVelocity)) {
      coreHex.setPower(1);
    } else {
      coreHex.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void farPowerAuto() {
    ((DcMotorEx) flywheel).setVelocity(farVelocity);
    servo.setPower(1);
    if (Math.abs(((DcMotorEx) flywheel).getVelocity() - farVelocity) <= shot_tolerance) {
      coreHex.setPower(1);
    } else {
      coreHex.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void setFlywheelVelocity() {
    if (gamepad1.dpad_up) {
      ((DcMotorEx) flywheel).setVelocity(0 - ShotVelocity);
    } else if (gamepad1.dpad_down) {
      ((DcMotorEx) flywheel).setVelocity(ShotVelocity);
    }
    if (gamepad1.left_bumper) {
      bankShotAuto();
    } else if (gamepad1.right_bumper) {
      farPowerAuto();
    }
    if (gamepad1.x) {
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    } else {
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    if (!gamepad1.ps && !gamepad1.a && !gamepad1.b) {
      servo.setPower(0);
    }
    if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.left_bumper && !gamepad1.right_bumper) {
      ((DcMotorEx) flywheel).setVelocity(0);
    }
    if (!gamepad1.b && !gamepad1.y && !gamepad1.a) {
      coreHex.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void splitStickArcadeDrive() {
    float y;
    float x;

    if (true) {
      y = gamepad2.left_stick_y;
      x = gamepad2.right_stick_x;
      x = (float) (x * Math.max(1 - gamepad1.right_trigger, slowSpeed));
      y = (float) (y * Math.max(1 - gamepad1.right_trigger, slowSpeed));
      backLeft.setPower(y - x);
      backRight.setPower(y + x);
      frontLeft.setPower(y - x);
      frontRight.setPower(y + x);
    }
  }
}

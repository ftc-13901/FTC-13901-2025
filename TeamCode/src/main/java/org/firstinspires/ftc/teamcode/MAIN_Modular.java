package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

@TeleOp(name = "MAIN_Modular")
public class MAIN_Modular extends LinearOpMode {

  private DcMotor flywheel;
  private DcMotor coreHex;
  private DcMotor backLeft;
  private DcMotor frontLeft;
  private CRServo servo;
  private VoltageSensor controlHubVoltageSensor;
  private DcMotor frontRight;
  private DcMotor backRight;
  private ServoImplEx righty;
  private IMU imu;
  private ServoImplEx lefty;

  private double speedScale;
  private int shotVelocity;
  private int farVelocity;
  private double slowSpeed;
  private double frontServoPosition;
  private int shotTolerance;
  private ElapsedTime servosIdleTimer;
  private double fullSpeedScale;

  @Override
  public void runOpMode() {
    initializeHardware();
    configureDrivetrain();
    waitForStart();

    while (opModeIsActive()) {
      if (controlHubVoltageSensor.getVoltage() > 7) {
        updateDriveMotors();
        updateFlywheelMotor();
        updateCoreHexMotor();
        updateFeederServo();
        updateFrontServos();
        updateTelemetry();
      } else {
        stopAllSubsystems();
        telemetry.addLine("battery low");
        telemetry.update();
      }
    }
  }

  private void initializeHardware() {
    flywheel = hardwareMap.get(DcMotor.class, "flywheel");
    coreHex = hardwareMap.get(DcMotor.class, "coreHex");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    servo = hardwareMap.get(CRServo.class, "servo");
    controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    righty = hardwareMap.get(ServoImplEx.class, "righty");
    imu = hardwareMap.get(IMU.class, "imu");
    lefty = hardwareMap.get(ServoImplEx.class, "lefty");

    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    flywheel.setDirection(DcMotor.Direction.REVERSE);
    coreHex.setDirection(DcMotor.Direction.FORWARD);
    backLeft.setDirection(DcMotor.Direction.REVERSE);
    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    servo.setPower(0);

    shotVelocity = 1300;
    farVelocity = 1500;
    slowSpeed = 0.25;
    shotTolerance = 80;
    ((DcMotorEx) flywheel).setVelocityPIDFCoefficients(1.0204, 0, 0.00035, 14.5);
  }

  private void configureDrivetrain() {
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    righty.setDirection(Servo.Direction.REVERSE);
    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    slowSpeed = 0.5;
    fullSpeedScale = 0.7;
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP)));

    servosIdleTimer = new ElapsedTime();
    lefty.setPosition(0);
    righty.setPosition(0);
    frontServoPosition = 0;
  }

  private void updateDriveMotors() {
    if (gamepad1.ps) {
      drive(0, 0, 0);
      coreHex.setPower(-1);
      ((DcMotorEx) flywheel).setVelocity(-3000);
      servo.setPower(1);
      return;
    }

    if (gamepad1.a) {
      imu.resetYaw();
    }

    speedScale = Math.max(1 - gamepad1.right_trigger, slowSpeed);

    if (gamepad1.x) {
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    } else {
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    drive(gamepad1.left_stick_y * speedScale, -(gamepad1.left_stick_x * speedScale), -(gamepad1.right_stick_x * speedScale));
  }

  private void updateFlywheelMotor() {
    if (gamepad1.ps) {
      return;
    }

    if (gamepad1.dpad_up) {
      ((DcMotorEx) flywheel).setVelocity(-shotVelocity);
    } else if (gamepad1.dpad_down) {
      ((DcMotorEx) flywheel).setVelocity(shotVelocity);
    } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
      ((DcMotorEx) flywheel).setVelocity(0);
    }
  }

  private void updateCoreHexMotor() {
    if (gamepad1.ps) {
      return;
    }

    if (gamepad1.left_bumper) {
      bankShotAuto();
    } else if (gamepad1.right_bumper) {
      farPowerAuto();
    } else if (gamepad1.y) {
      coreHex.setPower(-0.5);
    } else if (!gamepad1.b && !gamepad1.y && !gamepad1.a) {
      coreHex.setPower(0);
    }
  }

  private void updateFeederServo() {
    if (gamepad1.ps) {
      servo.setPower(1);
      return;
    }

    if (gamepad1.left_bumper || gamepad1.right_bumper) {
      return;
    }

    if (!gamepad1.ps && !gamepad1.a && !gamepad1.b) {
      servo.setPower(0);
    }
  }

  private void updateFrontServos() {
    telemetry.addData("leftyPos", lefty.getPosition());
    telemetry.addData("rightyPos", righty.getPosition());

    if (gamepad2.back || gamepad2.start) {
      lefty.setPwmEnable();
      righty.setPwmEnable();
      servosIdleTimer.reset();
    }

    if (gamepad2.start) {
      frontServoPosition = Math.min(1.0, frontServoPosition + 0.05);
    } else if (gamepad2.back) {
      frontServoPosition = Math.max(0.0, frontServoPosition - 0.05);
    }

    if (2000 < servosIdleTimer.milliseconds()) {
      lefty.setPwmDisable();
      righty.setPwmDisable();
    } else {
      lefty.setPosition(frontServoPosition);
      righty.setPosition(frontServoPosition);
    }
  }

  private void updateTelemetry() {
    telemetry.addData("flywheel velocity", ((DcMotorEx) flywheel).getVelocity());
    telemetry.addData("feeder", coreHex.getPower());
    telemetry.addData("battery", controlHubVoltageSensor.getVoltage());
    telemetry.addData("flywheel target velocity", shotVelocity);
    telemetry.addData("shot tolerance", shotTolerance);
    telemetry.addData("speedScale", speedScale);
    telemetry.update();
  }

  private void stopAllSubsystems() {
    drive(0, 0, 0);
    ((DcMotorEx) flywheel).setVelocity(0);
    coreHex.setPower(0);
    servo.setPower(0);
  }

  private void bankShotAuto() {
    ((DcMotorEx) flywheel).setVelocity(shotVelocity);
    coreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    servo.setPower(1);
    if (shotTolerance > Math.abs(((DcMotorEx) flywheel).getVelocity() - shotVelocity)) {
      coreHex.setPower(1);
    } else {
      coreHex.setPower(0);
    }
  }

  private void farPowerAuto() {
    ((DcMotorEx) flywheel).setVelocity(farVelocity);
    servo.setPower(1);
    if (Math.abs(((DcMotorEx) flywheel).getVelocity() - farVelocity) <= shotTolerance) {
      coreHex.setPower(1);
    } else {
      coreHex.setPower(0);
    }
  }

  private void drive(double forward, double right, double rotate) {
    double maxPower = 0;
    double frontLeftPower = forward + right + rotate;
    double frontRightPower = forward - (right + rotate);
    double backRightPower = forward + (right - rotate);
    double backLeftPower = forward - (right - rotate);

    maxPower = JavaUtil.maxOfList(JavaUtil.createListWith(
        Math.abs(frontLeftPower),
        Math.abs(frontRightPower),
        Math.abs(backLeftPower),
        Math.abs(backRightPower),
        maxPower));

    if (maxPower <= fullSpeedScale * 0.9) {
      maxPower = 1;
    }

    double maxSpeed = speedScale * fullSpeedScale;
    frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
    frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
    backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
    backRight.setPower(maxSpeed * (backRightPower / maxPower));
  }

  private void driveRelative(double forwardRelative, double rightRelative, int rotateRelative) {
    double theta = Math.atan2(forwardRelative, rightRelative) / Math.PI * 180;
    double r = Math.sqrt(forwardRelative * forwardRelative + rightRelative * rightRelative);
    theta = AngleUnit.DEGREES.normalize(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    drive(r * Math.sin(theta / 180 * Math.PI), r * Math.cos(theta / 180 * Math.PI), rotateRelative);
  }
}

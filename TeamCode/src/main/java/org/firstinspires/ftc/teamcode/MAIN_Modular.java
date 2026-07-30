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

  private boolean emergencyStopRequested;
  private boolean resetYawRequested;
  private boolean brakeDriveRequested;
  private boolean useRelativeDrive;
  private boolean flywheelForwardRequested;
  private boolean flywheelReverseRequested;
  private boolean bankShotRequested;
  private boolean farShotRequested;
  private boolean manualCoreHexRequested;
  private boolean stopFlywheelRequested;
  private boolean stopCoreHexRequested;
  private boolean stopFeederRequested;
  private boolean frontServoEnableRequested;
  private boolean frontServoIncreaseRequested;
  private boolean frontServoDecreaseRequested;
  private double driveForwardInput;
  private double driveStrafeInput;
  private double driveRotateInput;

  @Override
  public void runOpMode() {
    initializeHardware();
    configureDrivetrain();
    waitForStart();

    while (opModeIsActive()) {
      if (controlHubVoltageSensor.getVoltage() > 7) {
        updateControllerInputs();
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

  private void updateControllerInputs() {
    emergencyStopRequested = gamepad1.ps;
    resetYawRequested = gamepad1.a;
    brakeDriveRequested = gamepad1.x;
    useRelativeDrive = gamepad2.right_bumper;
    flywheelForwardRequested = gamepad1.dpad_up;
    flywheelReverseRequested = gamepad1.dpad_down;
    bankShotRequested = gamepad1.left_bumper;
    farShotRequested = gamepad1.right_bumper;
    manualCoreHexRequested = gamepad1.y;
    frontServoEnableRequested = gamepad2.back || gamepad2.start;
    frontServoIncreaseRequested = gamepad2.start;
    frontServoDecreaseRequested = gamepad2.back;

    driveForwardInput = gamepad1.left_stick_y;
    driveStrafeInput = -gamepad1.left_stick_x;
    driveRotateInput = -gamepad1.right_stick_x;
    speedScale = Math.max(1 - gamepad1.right_trigger, slowSpeed);

    stopFlywheelRequested = !flywheelForwardRequested && !flywheelReverseRequested && !bankShotRequested && !farShotRequested;
    stopCoreHexRequested = !bankShotRequested && !farShotRequested && !manualCoreHexRequested && !gamepad1.b && !gamepad1.y && !gamepad1.a;
    stopFeederRequested = !gamepad1.ps && !gamepad1.a && !gamepad1.b;
  }

  private void updateDriveMotors() {
    if (emergencyStopRequested) {
      drive(0, 0, 0);
      coreHex.setPower(-1);
      ((DcMotorEx) flywheel).setVelocity(-3000);
      servo.setPower(1);
      return;
    }

    if (resetYawRequested) {
      imu.resetYaw();
    }

    if (brakeDriveRequested) {
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    } else {
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    if (useRelativeDrive) {
      driveRelative(driveForwardInput * speedScale, driveStrafeInput * speedScale, (int) (driveRotateInput * speedScale));
    } else {
      drive(driveForwardInput * speedScale, driveStrafeInput * speedScale, driveRotateInput * speedScale);
    }
  }

  private void updateFlywheelMotor() {
    if (emergencyStopRequested) {
      return;
    }

    if (flywheelForwardRequested) {
      ((DcMotorEx) flywheel).setVelocity(-shotVelocity);
    } else if (flywheelReverseRequested) {
      ((DcMotorEx) flywheel).setVelocity(shotVelocity);
    } else if (stopFlywheelRequested) {
      ((DcMotorEx) flywheel).setVelocity(0);
    }
  }

  private void updateCoreHexMotor() {
    if (emergencyStopRequested) {
      return;
    }

    if (bankShotRequested) {
      bankShotAuto();
    } else if (farShotRequested) {
      farPowerAuto();
    } else if (manualCoreHexRequested) {
      coreHex.setPower(-0.5);
    } else if (stopCoreHexRequested) {
      coreHex.setPower(0);
    }
  }

  private void updateFeederServo() {
    if (emergencyStopRequested) {
      servo.setPower(1);
      return;
    }

    if (bankShotRequested || farShotRequested) {
      return;
    }

    if (stopFeederRequested) {
      servo.setPower(0);
    }
  }

  private void updateFrontServos() {
    telemetry.addData("leftyPos", lefty.getPosition());
    telemetry.addData("rightyPos", righty.getPosition());

    if (frontServoEnableRequested) {
      lefty.setPwmEnable();
      righty.setPwmEnable();
      servosIdleTimer.reset();
    }

    if (frontServoIncreaseRequested) {
      frontServoPosition = Math.min(1.0, frontServoPosition + 0.05);
    } else if (frontServoDecreaseRequested) {
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

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.analog.adis16448.frc.ADIS16448_IMU;

public class Robot extends TimedRobot {
  // Victors
  WPI_VictorSPX _frontTLeftMotor;
  WPI_VictorSPX _frontTRightMotor;
  WPI_VictorSPX _rearTRightMotor;
  WPI_VictorSPX _rearTLeftMotor;
  WPI_VictorSPX _intakeUpperMotor;
  WPI_VictorSPX _intakeLowerMotor;

  // Talons
  WPI_TalonSRX _frontLifterOne;
  WPI_TalonSRX _frontLifterTwo;
  WPI_TalonSRX _rearLifterMotor;
  WPI_TalonSRX _intakeLifterMotor;
  SpeedControllerGroup frontLifterMotors;
  MecanumDrive _mDrive;
  Joystick _joy1;
  Joystick _joy2;
  private UsbCamera camera;

  // Network Tables
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTable table;

  // Serial Port Information
  SerialPort theThePort;

  // Line Tracker
  DigitalInput lineTracker0;
  DigitalInput lineTracker1;
  DigitalInput lineTracker2;
  DigitalInput lineTracker3;
  DigitalInput lineTracker4;

  // Lifter Logic
  double frontLiftSpeed = 0.50;
  double rearLiftSpeed = 0;
  double maxFrontLiftSpeed = 0.9;
  double maxRearLiftSpeed = 0.9;

  // 10 Degrees of Freedom
  ADIS16448_IMU imu;
  double zDegree = 0;
  double xDegree = 0;
  double yDegree = 0;
  boolean didItAlready = false;
  boolean imuIsWorkingCorrectly = true; // IMU is Working or Not

  // Line Tracker Values
  double pastZDegree = zDegree;
  int zDegreeIterations = 0;
  double targetDegree = 0;
  double rotationCounter = 1;
  double rotation = 0;
  double forwardMotion = 0;
  boolean lTrack0 = false;
  boolean lTrack1 = false;
  boolean lTrack2 = false;
  boolean lTrack3 = false;
  boolean lTrack4 = false;
  boolean autoTrackingEnabled = false;

  // Sections of code to include or exclude
  boolean nTables = false; // Network Tables in Use
  boolean cServer = false; // Camera Server
  boolean jCam = false; // Jevois Camera
  boolean lTrack = false; // Line Tracker
  boolean tenDegrees = true; // 10 degrees of freedom
  boolean pneumatics = false; // Pneumatics System

  // Timer
  Timer robotTimer = new Timer();

  // Pneumatics
  DoubleSolenoid pneuAction;

  @Override
  public void robotInit() {

    robotTimer.start(); // Start the timer for IMU Calibration Safeguard.

    // Setup the joystick
    try {
      _joy1 = new Joystick(0);
    } catch (Exception ex) {
    }
    try {
      _joy2 = new Joystick(1);
    } catch (Exception ex) {
    }

    // Setup the Drive System
    _frontTLeftMotor = new WPI_VictorSPX(13);
    _frontTRightMotor = new WPI_VictorSPX(12);
    _rearTRightMotor = new WPI_VictorSPX(11);
    _rearTLeftMotor = new WPI_VictorSPX(10);
    // Invert all the motors, they're probably wired wrong
    _frontTLeftMotor.setInverted(true);
    _frontTRightMotor.setInverted(true);
    _rearTLeftMotor.setInverted(true);
    _rearTRightMotor.setInverted(true);
    _mDrive = new MecanumDrive(_frontTLeftMotor, _rearTLeftMotor, _frontTRightMotor, _rearTRightMotor);

    // Create front Lifter motors
    _frontLifterOne = new WPI_TalonSRX(23);
    _frontLifterTwo = new WPI_TalonSRX(22);
    // We would like these to brake when set to zero speed
    _frontLifterOne.setNeutralMode(NeutralMode.Brake);
    _frontLifterTwo.setNeutralMode(NeutralMode.Brake);
    frontLifterMotors = new SpeedControllerGroup(_frontLifterOne, _frontLifterTwo);

    // Create rear Lifter Motors
    _rearLifterMotor = new WPI_TalonSRX(20);
    // We want this moter to break when set to zero speed
    _rearLifterMotor.setNeutralMode(NeutralMode.Brake);

    // Create the Intake Motors
    _intakeLifterMotor = new WPI_TalonSRX(21);
    _intakeLowerMotor = new WPI_VictorSPX(31);
    _intakeUpperMotor = new WPI_VictorSPX(30);

    // Create the line tracker sensors
    if (lTrack) {
      try {
        lineTracker0 = new DigitalInput(0);
      } catch (Exception ex) {
      }
      try {
        lineTracker1 = new DigitalInput(1);
      } catch (Exception ex) {
      }
      try {
        lineTracker2 = new DigitalInput(2);
      } catch (Exception ex) {
      }
      try {
        lineTracker3 = new DigitalInput(3);
      } catch (Exception ex) {
      }
      try {
        lineTracker4 = new DigitalInput(4);
      } catch (Exception ex) {
      }
    }

    // Create the 10 Degrees of Freedom
    if (tenDegrees) {
      try {
        imu = new ADIS16448_IMU();
      } catch (Exception ex) {
        imu = new ADIS16448_IMU();
      }
      try {
        imu.reset();
        imu.calibrate();
      } catch (Exception ex) {
      }
    }

    // If our camera is onboard the roborio - This year it's not
    if (cServer) {
      try {
        camera = CameraServer.getInstance().startAutomaticCapture(0);
      } catch (Exception ex) {
      }
      if (camera == null) {
        camera = null; // This will prevent a "Warning" error during compilation
      }
    }

    // We might need network tables in our near future, but not yet
    if (nTables) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      table = inst.getTable("deepSpace");
      xEntry = table.getEntry("X");
      yEntry = table.getEntry("Y");
    }

    // Serial Port Logic for our JeVois Camera System
    if (jCam) {
      try {
        theThePort = new SerialPort(115200, Port.kUSB);
      } catch (Exception e) {
        // jCamString = e.toString();
      }
      int retval = 0;
      if (theThePort != null) {
        retval = theThePort.writeString("ping\n");
      }
      if (retval > 0) {
        SmartDashboard.putString("Error", "The the error: " + retval);
        SmartDashboard.putString("Error2", theThePort.readString());
      }
    }

    // Create our Pneumatics controls
    if (pneumatics) {
      pneuAction = new DoubleSolenoid(8, 9);
    }
  }

  @Override
  public void robotPeriodic() {
    // Perform a full IMU reset and calibration when both "Start" pressed
    // This might take up to 9 seconds
    if (_joy1.getRawButton(8) && _joy2.getRawButton(8)) {
      manualImuCalibration();
    }
    if (!didItAlready) {
      imuCalibration();
    }

    // Reset the imu when the "Y" yellow button is pressed
    if (_joy1.getRawButton(3) && tenDegrees) {
      imu.reset();
      imuIsWorkingCorrectly = true;
      SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    }

    if (tenDegrees) {
      SmartDashboard.putNumber("zDegree", zDegree);
      SmartDashboard.putNumber("xDegree", xDegree);
      SmartDashboard.putNumber("yDegree", yDegree);
      SmartDashboard.putNumber("targetDegree", targetDegree);
      SmartDashboard.putBoolean("Line Tracker 0", lTrack0);
      SmartDashboard.putBoolean("Line Tracker 1", lTrack1);
      SmartDashboard.putBoolean("Line Tracker 2", lTrack2);
      SmartDashboard.putBoolean("Line Tracker 3", lTrack3);
      SmartDashboard.putBoolean("Line Tracker 4", lTrack4);
      SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
      SmartDashboard.putBoolean("Auto Tracking", autoTrackingEnabled);
      SmartDashboard.putNumber("frontspeed", frontLiftSpeed);
      SmartDashboard.putNumber("rearspeed", rearLiftSpeed);
    }
    // Turn autoTracking on/off
    if (_joy2.getRawButton(4) || _joy1.getRawButton(4)) {
      if (autoTrackingEnabled) {
        autoTrackingEnabled = false;
      } else {
        autoTrackingEnabled = true;
      }
    }
  }

  // This, before match has begun, should go periodically until we did it.
  private void imuCalibration() {
    if (Timer.getMatchTime() > 0) {
      robotTimer.stop();
      didItAlready = true;
    } else if (robotTimer.get() > 300.0) // if 5+ mins have passed since power on
    {
      didItAlready = true;
      manualImuCalibration();
      robotTimer.stop();
    }
  }

  // Manually calibrate the IMU whenever, disable or change for testing
  private void manualImuCalibration() {
    imuIsWorkingCorrectly = false;
    SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    try {
      imu.reset();
      imu.calibrate();
    } catch (Exception e) {
    }
    imuIsWorkingCorrectly = true;
    SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    zDegreeIterations = 0;
  }

  public double find45Degree(double zDegree) {
    double retDoub = -1;
    int povVal1 = _joy1.getPOV();
    int povVal2 = _joy2.getPOV();
    if (povVal2 >= 0) {
      retDoub = povVal2;
    } else if (povVal1 >= 0) {
      retDoub = povVal1;
    } else {
      int plusOne = (int) zDegree;
      int minusOne = (int) zDegree;
      while (retDoub < 0) {
        switch (plusOne) {
        case 0:
          retDoub = 0;
          break;
        case 45:
          retDoub = 45;
          break;
        case 90:
          retDoub = 90;
          break;
        case 135:
          retDoub = 135;
          break;
        case 180:
          retDoub = 180;
          break;
        case 225:
          retDoub = 225;
          break;
        case 270:
          retDoub = 270;
          break;
        case 315:
          retDoub = 315;
          break;
        case 360:
          retDoub = 0;
          break;
        default:
          break;
        }
        switch (minusOne) {
        case 0:
          retDoub = 0;
          break;
        case 45:
          retDoub = 45;
          break;
        case 90:
          retDoub = 90;
          break;
        case 135:
          retDoub = 135;
          break;
        case 180:
          retDoub = 180;
          break;
        case 225:
          retDoub = 225;
          break;
        case 270:
          retDoub = 270;
          break;
        case 315:
          retDoub = 315;
          break;
        case 360:
          retDoub = 0;
          break;
        default:
          break;
        }
        if (retDoub < 0) {
          plusOne++;
          minusOne--;
        }
      }
    }
    return retDoub;
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    forwardMotion = _joy1.getRawAxis(1) * -1;

    // Platform Climb Logic
    rearLiftSpeed = frontLiftSpeed * 0.7;
    yDegree = Math.round(imu.getAngleY()) % 360;
    if (yDegree > 5 && yDegree < 180) {
      rearLiftSpeed = rearLiftSpeed + 0.4; // Increase Speed to rear if front is too fast
    } else if (yDegree < -5.0) {
      rearLiftSpeed = 0.0; // Turn off rear if front is too slow
    } else if (yDegree < 0) {
      rearLiftSpeed = rearLiftSpeed - 0.25; // Decrease speed to rear if front is lagging slightly
    }
    if (rearLiftSpeed > maxRearLiftSpeed) {
      rearLiftSpeed = maxRearLiftSpeed; // Enforce max speed limits
    }
    if (frontLiftSpeed > maxFrontLiftSpeed) {
      frontLiftSpeed = maxFrontLiftSpeed; // Enforce max speed limits
    }
    // This is our platform climb at the end
    if (_joy1.getRawButton(2)) {
      frontLifterMotors.set(frontLiftSpeed);
      _rearLifterMotor.set(rearLiftSpeed * -1.0);
      if (forwardMotion < 0.5 && forwardMotion > -0.5) {
        forwardMotion = 0.2; // This should cause a slow forward wheel spin while climbing
      }
    } else if (_joy1.getRawButton(1)) {
      frontLifterMotors.set(frontLiftSpeed * -1.0);
      _rearLifterMotor.set(rearLiftSpeed);
    } else if (_joy1.getRawButton(6)) {
      frontLifterMotors.set(0.4);
      _rearLifterMotor.set(0.0);
    } else if (_joy1.getRawButton(5)) {
      frontLifterMotors.set(-0.4);
      _rearLifterMotor.set(0.0);
    } else if (Math.abs(_joy1.getRawAxis(2)) > .2) {
      frontLifterMotors.set(0.0);
      _rearLifterMotor.set(0.3);
    } else if (Math.abs(_joy1.getRawAxis(3)) > .2) {
      frontLifterMotors.set(0.0);
      _rearLifterMotor.set(-0.3);
    } else {
      frontLifterMotors.set(0.0);
      _rearLifterMotor.set(0.0);
    }

    // Intake Logic Begins Here
    double lowerInTake = _joy2.getRawAxis(1);
    if (lowerInTake > 0.05 || lowerInTake < -0.05) {
      _intakeLifterMotor.set(lowerInTake);
    } else {
      _intakeLifterMotor.set(0);
    }

    double ballInTakeIn = _joy2.getRawAxis(2);
    double ballInTakeOut = _joy2.getRawAxis(3);
    if(ballInTakeIn > 0.05 || ballInTakeIn < -0.05) {
      _intakeLowerMotor.set(ballInTakeIn);
      _intakeUpperMotor.set(-ballInTakeIn);
    } else if(ballInTakeOut > 0.05 || ballInTakeOut < -0.05) {
      _intakeLowerMotor.set(-ballInTakeOut);
      _intakeUpperMotor.set(ballInTakeOut);
    } else {
      _intakeLowerMotor.set(0);
      _intakeUpperMotor.set(0);
    }

    // Setup Stafe values
    double strafe = 0;
    if (_joy1.getRawAxis(0) > 0.1 || _joy1.getRawAxis(0) < -0.1) {
      strafe = _joy1.getRawAxis(0);
    } else if (_joy1.getRawAxis(2) > 0.1) {
      strafe = _joy1.getRawAxis(2) * -1.0;
    } else if (_joy1.getRawAxis(3) > 0.1) {
      strafe = _joy1.getRawAxis(3);
    }
    if (tenDegrees) {
      zDegree = Math.round(imu.getAngleZ()) % 360;
      xDegree = Math.round(imu.getAngleX()) % 360;
      yDegree = Math.round(imu.getAngleY()) % 360;
      if (zDegree < 0) {
        zDegree += 360;
      }
      targetDegree = find45Degree(zDegree);
      if (lTrack) {
        lTrack0 = lineTracker0.get();
        lTrack1 = lineTracker1.get();
        lTrack2 = lineTracker2.get();
        lTrack3 = lineTracker3.get();
        lTrack4 = lineTracker4.get();
      }
      if (autoTrackingEnabled) { // Line Tracker Enabled
        rotation = _joy1.getRawAxis(4);
        if (lTrack0) {
          rotation = rotation + turnSpeed(0.3);
          strafe = strafe + 0.6;
          _mDrive.driveCartesian(strafe, forwardMotion, rotation, 0);
        } else if (lTrack4) {
          rotation = rotation + turnSpeed(0.3);
          strafe = strafe - 0.6;
          _mDrive.driveCartesian(strafe, forwardMotion, rotation, 0);
        } else if (lTrack1) {
          rotation = rotation + turnSpeed(0.2);
          strafe = strafe + 0.4;
          _mDrive.driveCartesian(strafe, forwardMotion, rotation, 0);
        } else if (lTrack3) {
          rotation = rotation + turnSpeed(0.2);
          strafe = strafe - 0.4;
          _mDrive.driveCartesian(strafe, forwardMotion, rotation, 0);
        } else if (lTrack2) {
          rotation = rotation + turnSpeed(0.1);
          _mDrive.driveCartesian(0, forwardMotion, rotation, 0);
        } else {
          _mDrive.driveCartesian(strafe, forwardMotion, _joy1.getRawAxis(4), 0);
        }
      } else {
        _mDrive.driveCartesian(strafe, forwardMotion, _joy1.getRawAxis(4), 0);
      }
    } else {
      // The the mecanum drive is listed below
      _mDrive.driveCartesian(strafe, forwardMotion, _joy1.getRawAxis(4), 0);
    }
    if (_joy1.getRawAxis(4) < -0.2 || _joy1.getRawAxis(4) > 0.2) {
      if (pastZDegree == zDegree) {
        zDegreeIterations++;
        if (zDegreeIterations > 15) {
          imuIsWorkingCorrectly = false; // We have a real Problem
        }
      } else {
        pastZDegree = zDegree;
        zDegreeIterations = 0;
      }
      // if (pneumatics) {
      // if (_joy1.getRawButton(5) || _joy2.getRawButton(5)) {
      // pneuAction.set(DoubleSolenoid.Value.kReverse);
      // } else {
      // pneuAction.set(DoubleSolenoid.Value.kForward);
      // }
      // }
    }
  }

  public double turnSpeed(double fullspeed) {
    double returnSpeed = fullspeed;
    if (targetDegree == 0 && zDegree > 270) {
      returnSpeed = returnSpeed * -1.0; // we are overlapping zero
    }
    if (zDegree > targetDegree) {
      returnSpeed = returnSpeed * -1.0;
    }
    return returnSpeed;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

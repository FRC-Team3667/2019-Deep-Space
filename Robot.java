package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
import com.analog.adis16448.frc.ADIS16448_IMU;

public class Robot extends TimedRobot {
  WPI_TalonSRX _frontTLeftMotor = null;
  WPI_TalonSRX _frontTRightMotor = null;
  WPI_TalonSRX _rearTRightMotor = null;
  WPI_TalonSRX _rearTLeftMotor = null;
  WPI_VictorSPX _frontVLeftMotor = null;
  WPI_VictorSPX _frontVRightMotor = null;
  WPI_VictorSPX _rearVRightMotor = null;
  WPI_VictorSPX _rearVLeftMotor = null;
  SpeedControllerGroup leftMotors = null;
  SpeedControllerGroup rightMotors = null;
  DifferentialDrive _dDrive = null;
  MecanumDrive _mDrive = null;
  Joystick _joy1 = null;
  Joystick _joy2 = null;
  private UsbCamera camera = null;

  // Network Tables
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTable table;

  // Serial Port Information
  SerialPort theThePort = null;

  // Line Tracker
  DigitalInput lineTracker0 = null;
  DigitalInput lineTracker1 = null;
  DigitalInput lineTracker2 = null;
  DigitalInput lineTracker3 = null;
  DigitalInput lineTracker4 = null;

  // 10 Degrees of Freedom
  ADIS16448_IMU imu;

  double zDegree = 0;
  double pastZDegree = zDegree;
  int zDegreeIterations = 0;
  double targetDegree = 0;
  double origLine2Degree = 0;
  double rotationCounter = 1;
  double rotation = 0;
  double forwardMotion = 0;
  boolean lTrack0 = false;
  boolean lTrack1 = false;
  boolean lTrack2 = false;
  boolean lTrack3 = false;
  boolean lTrack4 = false;
  boolean autoTrackingEnabled = true;

  // Sections of code to include or exclude
  boolean nTables = false; // Network Tables in Use
  boolean mDrive = true; // Mecanum Drive
  boolean dDrive = false; // Differential Drive
  boolean cServer = true; // Camera Server
  boolean jCam = false; // Jevois Camera
  boolean lTrack = true; // Line Tracker
  boolean tenDegrees = true; // 10 degrees of freedom
  // CAUSE OF THE CRCS

  String jCamString = " ";

  // Timer
  Timer robotTimer = new Timer();

  // Guard It Safe, this's the IMU calibration one
  boolean didItAlready = false;
  boolean imuIsWorkingCorrectly = true; // IMU is Working or Not

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    robotTimer.start(); // Start the timer for IMU Calibration Guard It Safes.

    // Setup the joystick
    try {
      _joy1 = new Joystick(0);
    } catch (Exception e0) {
    }
    try {
      _joy2 = new Joystick(1);
    } catch (Exception e1) {
    }

    // Setup the Drive System
    if (mDrive) {
      _frontVLeftMotor = new WPI_VictorSPX(13);
      _frontVRightMotor = new WPI_VictorSPX(12);
      _rearVRightMotor = new WPI_VictorSPX(11);
      _rearVLeftMotor = new WPI_VictorSPX(10);
      _mDrive = new MecanumDrive(_frontVLeftMotor, _rearVLeftMotor, _frontVRightMotor, _rearVRightMotor);
    } else if (dDrive) {
      _frontTLeftMotor = new WPI_TalonSRX(13);
      _frontTRightMotor = new WPI_TalonSRX(12);
      _rearTRightMotor = new WPI_TalonSRX(11);
      _rearTLeftMotor = new WPI_TalonSRX(10);
      leftMotors = new SpeedControllerGroup(_frontTLeftMotor, _rearTLeftMotor);
      rightMotors = new SpeedControllerGroup(_frontTRightMotor, _rearTRightMotor);
      _dDrive = new DifferentialDrive(leftMotors, rightMotors);
    }

    if (lTrack) {
      try {
        lineTracker0 = new DigitalInput(0);
        lineTracker1 = new DigitalInput(1);
        lineTracker2 = new DigitalInput(2);
        lineTracker3 = new DigitalInput(3);
        lineTracker4 = new DigitalInput(4);
      } catch (Exception ex) {
      }
    }

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

    if (tenDegrees) {
      // 10 Degrees of Freedom
      try {
        imu = new ADIS16448_IMU();
      } catch (Exception e) {
        imu = new ADIS16448_IMU();
      }
      try {
        imu.calibrate();
        imu.reset();
      } catch (Exception e) {

      }
    }

    if (cServer) {
      try {
        camera = CameraServer.getInstance().startAutomaticCapture(0);
      } catch (Exception e) {
      }
      if (camera == null) { // Don't ask why!
        camera = null; // Seriously, don't!
      }
    }

    if (nTables) {
      // Network Tables
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      table = inst.getTable("deepSpace");
      xEntry = table.getEntry("X");
      yEntry = table.getEntry("Y");
    }

    // Serial Port Logic
    if (jCam) {
      try {
        theThePort = new SerialPort(115200, Port.kUSB);
      } catch (Exception e) {
        jCamString = e.toString();
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

    SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // To calibrate imu manually, press start on both controllers. Only usable
    // before auton.
    // NOTE: The 3667 Programmers are not liable for any mistakes that are made if
    // you accidentally
    // press these buttons. Oh yes, and if you think it was our fault we will find
    // you and you will
    // regret ever thinking so.
    if (_joy1.getRawButton(8) && _joy2.getRawButton(8)) {
      manualImuCalibration();
    }
    if (!didItAlready) {
      imuCalibration();
    }

    // Network Table Test Work
    if (nTables) {
      if (_joy1.getRawButton(2)) {
        double xDouble = Math.random();
        double yDouble = Math.random();
        xEntry.setDouble(xDouble);
        yEntry.setDouble(yDouble);
      }
    }

    if (_joy1.getRawButton(3) && tenDegrees) {
      imu.reset();
      imuIsWorkingCorrectly = true;
      SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    }

    if (mDrive) {
      if (tenDegrees) {
        SmartDashboard.putNumber("zDegree", zDegree);
        SmartDashboard.putNumber("targetDegree", targetDegree);
        SmartDashboard.putBoolean("Line Tracker 0", lTrack0);
        SmartDashboard.putBoolean("Line Tracker 1", lTrack1);
        SmartDashboard.putBoolean("Line Tracker 2", lTrack2);
        SmartDashboard.putBoolean("Line Tracker 3", lTrack3);
        SmartDashboard.putBoolean("Line Tracker 4", lTrack4);
        SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
        SmartDashboard.putBoolean("Auto Tracking", autoTrackingEnabled);
      }
    }

    if (_joy2.getRawButton(4) || _joy1.getRawButton(4))
      if (autoTrackingEnabled) {
        autoTrackingEnabled = false;
      } else {
        autoTrackingEnabled = true;
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
      imu.calibrate();
      imu.reset();
    } catch (Exception e) {
    }
    imuIsWorkingCorrectly = true;
    SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    zDegreeIterations = 0;
  }

  // Attribute a near-gradial speed change
  public double gradientSpeed(double fullspeed, double originalLocation, double desiredLocation,
      double currentLocation) {
    double halfSpeed = fullspeed / 2;
    if (halfSpeed < .1) {
      halfSpeed = 0.1;
    }
    double slowSpeed = (fullspeed / 4 * 3);
    if (slowSpeed < .1) {
      slowSpeed = 0.1;
    }
    double returnSpeed = fullspeed;
    if (origLine2Degree == -0.0001) {
      origLine2Degree = currentLocation;
    }
    if (desiredLocation >= originalLocation) {
      if (currentLocation > (desiredLocation - originalLocation) / 2) {
        returnSpeed = halfSpeed;
      }
      if (currentLocation > ((desiredLocation - originalLocation) / 4) * 3) {
        returnSpeed = slowSpeed;
      }
    } else {
      if (currentLocation < (originalLocation - desiredLocation) / 2) {
        returnSpeed = halfSpeed;
      }
      if (currentLocation < ((originalLocation - desiredLocation) / 4) * 3) {
        returnSpeed = slowSpeed;
      }
    }

    // At this moment, when you're wanting to go to 0, we must tell you:
    if (targetDegree == 0 && zDegree > 270) {
      returnSpeed = returnSpeed * -1.0; // negative * negative
    }
    if (zDegree > targetDegree) {
      returnSpeed = returnSpeed * -1.0;
    }
    return returnSpeed;
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
    // Setup Stafe values
    double strafe = 0;
    if (_joy1.getRawAxis(0) > 0.1 || _joy1.getRawAxis(0) < -0.1) {
      strafe = _joy1.getRawAxis(0);
    } else if (_joy1.getRawAxis(2) > 0.1) {
      strafe = _joy1.getRawAxis(2) * -1.0;
    } else if (_joy1.getRawAxis(3) > 0.1) {
      strafe = _joy1.getRawAxis(3);
    }
    if (mDrive) {
      if (tenDegrees) {
        zDegree = Math.round(imu.getAngleZ()) % 360;
        if (zDegree < 0) {
          zDegree += 360;
        }
        targetDegree = find45Degree(zDegree);
        lTrack0 = lineTracker0.get();
        lTrack1 = lineTracker1.get();
        lTrack2 = lineTracker2.get();
        lTrack3 = lineTracker3.get();
        lTrack4 = lineTracker4.get();
        double forwardMotion = _joy1.getRawAxis(1) * -1;
        if (autoTrackingEnabled) { // Line Tracker Enabled
          rotation = gradientSpeed(0.3, origLine2Degree, targetDegree, zDegree);
          rotation = rotation + _joy1.getRawAxis(4);
          if (lTrack0) {
            strafe = strafe + 0.6;
            _mDrive.driveCartesian(strafe, forwardMotion, rotation, 0);
          } else if (lTrack4) {
            strafe = strafe - 0.6;
            _mDrive.driveCartesian(strafe, forwardMotion, rotation, 0);
          } else if (lTrack1) {
            strafe = strafe + 0.4;
            _mDrive.driveCartesian(strafe, forwardMotion, rotation, 0);
          } else if (lTrack3) {
            strafe = strafe - 0.4;
            _mDrive.driveCartesian(strafe, forwardMotion, rotation, 0);
          } else if (lTrack2) {
            if (zDegree > targetDegree) {
              _mDrive.driveCartesian(0, forwardMotion, 0.15 * -1, 0);
            } else if (zDegree < targetDegree) {
              _mDrive.driveCartesian(0, forwardMotion, 0.15, 0);
            }
          } else {
            _mDrive.driveCartesian(strafe, forwardMotion, _joy1.getRawAxis(4), 0);
          }
        } else {
          _mDrive.driveCartesian(strafe, forwardMotion, _joy1.getRawAxis(4), 0);
        }
      } else {
        // The the mecanum drive is listed below
        if (mDrive) {
          _mDrive.driveCartesian(strafe, _joy1.getRawAxis(1) * -1, _joy1.getRawAxis(4), 0);
        }
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
      }
    }

    // The the Network Table cool Code is within the if Statement
    if (dDrive) {
      if (_joy1.getRawButton(1)) {
        double xDouble = 0;
        xDouble = xEntry.getDouble(xDouble);
        double yDouble = 0;
        yDouble = yEntry.getDouble(yDouble);
        _dDrive.arcadeDrive(xDouble * -1.0, yDouble);
      } else {
        _dDrive.arcadeDrive(_joy1.getRawAxis(1) * -1.0, _joy1.getRawAxis(4));
      }
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

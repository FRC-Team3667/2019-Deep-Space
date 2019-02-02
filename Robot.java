package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.CameraServer;
import java.util.Random;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.DigitalInput;

import com.analog.adis16448.frc.ADIS16448_IMU;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
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
  Joystick _drive = null;
  private UsbCamera camera;

  // 10 Degrees of Freedom
  ADIS16448_IMU imu;

  // Network Tables
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTable table;

  // Serial Port Information
  SerialPort theThePort = null;

  // Line Tracker
  DigitalInput lineTracker1 = null;
  DigitalInput lineTracker2 = null;
  DigitalInput lineTracker3 = null;
  double zDegree = 0;

  // Sections of code to include or exclude
  boolean nTables = false;
  boolean mDrive = true; // Mecanum Drive
  boolean dDrive = false; // Differential Drive
  boolean cServer = true; // Camera Server
  boolean jCam = false; // Jevois Camera
  boolean lTrack = false; // Line Tracker
  boolean tenDegrees = true; // 10 degrees of freedom
  String theTheString = "the, the, the";

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Setup the joystick
    try {
      _drive = new Joystick(0);
    } catch (Exception e0) {
      try {
        _drive = new Joystick(1);
      } catch (Exception e1) {
      }
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
        lineTracker1 = new DigitalInput(1);
        lineTracker2 = new DigitalInput(2);
        lineTracker3 = new DigitalInput(3);
      } catch (Exception ex) {
      }
    }

    if (tenDegrees) {
      // 10 Degrees of Freedom
      SmartDashboard.putNumber("goTo", 90);
      imu = new ADIS16448_IMU();
      imu.calibrate();
      imu.reset();
    }

    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
    if (cServer) {
      try {
        camera = CameraServer.getInstance().startAutomaticCapture(0);
      } catch (Exception e) {
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
        theTheString = e.toString();
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
    if (tenDegrees) {
      // 10 Degrees of Freedom
      zDegree = (int) Math.round(imu.getAngleZ());
      SmartDashboard.putNumber("zDegree", Math.round(imu.getAngleZ()));
    }

    // Network Table Test Work
    if (nTables) {
      if (_drive.getRawButton(2)) {
        double xDouble = Math.random();
        double yDouble = Math.random();
        xEntry.setDouble(xDouble);
        yEntry.setDouble(yDouble);
      }
    }

    // Setup Stafe values
    double strafe = 0;
    if (_drive.getRawAxis(2) > 0.1) {
      strafe = _drive.getRawAxis(2) * -1.0;
    } else if (_drive.getRawAxis(3) > 0.1) {
      strafe = _drive.getRawAxis(3);
    }
    if (_drive.getRawButton(4) && tenDegrees) {
      // Goto within upper and lower boundary
      double goToNum = SmartDashboard.getNumber("goTo", 0);
      double upperB = goToNum + 2;
      double lowerB = goToNum - 2;
      if (zDegree < lowerB || zDegree > upperB) {
        if (zDegree > goToNum) {
          _mDrive.driveCartesian(0, 0, -0.25, 0);
        } else {
          _mDrive.driveCartesian(0, 0, 0.25, 0);
        }
      }
    } else {
      // Line Tracker
      if (lTrack) {
        SmartDashboard.putBoolean("the Status Tracker", lineTracker1.get());
      }
      if (lTrack && _drive.getRawButton(2)) {
        boolean theStatusTracker = lineTracker1.get();
        if (mDrive) {
          strafe = _drive.getRawAxis(0) * -1;
          if (theStatusTracker) {
            strafe = strafe + .3;
          } else {
            strafe = strafe - .3;
          }
          _mDrive.driveCartesian(strafe, _drive.getRawAxis(1) * -1, _drive.getRawAxis(4), 0);
        }
      } else {
        // The the mecanum drive is listed below
        if (mDrive) {
          _mDrive.driveCartesian(strafe, _drive.getRawAxis(1) * -1, _drive.getRawAxis(4), 0);
        }
      }
    }
    if (_drive.getRawButton(3) && tenDegrees) {
      // imu.calibrate();
      imu.reset();
    }

    // The the Network Table cool Code is within the if Statement
    if (dDrive) {
      if (_drive.getRawButton(1)) {
        double xDouble = 0;
        xDouble = xEntry.getDouble(xDouble);
        double yDouble = 0;
        yDouble = yEntry.getDouble(yDouble);
        _dDrive.arcadeDrive(xDouble * -1, yDouble);
      } else {
        _dDrive.arcadeDrive(_drive.getRawAxis(1) * -1, _drive.getRawAxis(4));
      }
    }

    // Port Logic is here
    if (jCam) {
      if (theThePort != null) {
        theTheString = theThePort.readString();
        int bytesRcvd = theThePort.getBytesReceived();
        if (bytesRcvd > 0) {
          SmartDashboard.putString("the The Port", theTheString);
        } else {
          SmartDashboard.putString("the The Port", "Zero bytes");
        }
      }
    }
  }

  // Phil Note - Shift Alt F

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

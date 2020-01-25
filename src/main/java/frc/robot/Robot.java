/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  WPI_TalonSRX talon = new WPI_TalonSRX(35);
  Joystick Gamepad0 = new Joystick(0);
  Joystick Gamepad1 = new Joystick(1);

  CANSparkMax driveLeft = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax driveRight = new CANSparkMax(10, MotorType.kBrushless);
  // WPI_TalonSRX bottomLeftDrive = new WPI_TalonSRX(35);

  DifferentialDrive asdf = new DifferentialDrive(driveLeft, driveRight);

  PigeonIMU pigeon = new PigeonIMU(50);
 

  public double xCube;
  public double Kp = -0.6;
  public double min_command = 0;

  double p = 0.6;
  double d = 0.0;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    talon.configFeedbackNotContinuous(true, 10);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    talon.configPeakOutputForward(1, 10);
    talon.configPeakOutputReverse(-1, 10);

    SmartDashboard.putNumber("P", p);
    SmartDashboard.putNumber("D", d);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
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
    double currentPos = talon.getSelectedSensorPosition();

    PIDController pid = new PIDController(p, 0, d);
    
    double[] ypr = new double[3];
    double[] xyz = new double[3];
    pigeon.getYawPitchRoll(ypr);
    pigeon.getAccelerometerAngles(xyz);
    SmartDashboard.putNumber("Yaw", ypr[0]);
    SmartDashboard.putNumber("x", xyz[0]);
    SmartDashboard.putNumber("y", xyz[1]);
    SmartDashboard.putNumber("z", xyz[2]);

    p = SmartDashboard.getNumber("P", 0.6);
    d = SmartDashboard.getNumber("d", 0.0);

    asdf.tankDrive(Gamepad0.getThrottle(), Gamepad0.getY());

    SmartDashboard.putNumber("Turret Position", talon.getSelectedSensorPosition());

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    if (Gamepad1.getRawButton(2)) {
      table.getEntry("ledMode").setNumber(1);
    }
    else {
      table.getEntry("ledMode").setNumber(3);
    }
  
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);
    
    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("tv", v);

    double turretHome = 2348;
    double turretLeftStop = 1120;
    double turretRightStop = 3560;

    talon.set(Gamepad1.getThrottle());

    double distance = 60.25/(Math.tan(degToRad(26.85) + degToRad(y)));
    // power port height divided by tangent of angle of limelight + limelight y offset
    
    SmartDashboard.putNumber("Distance", distance);

    // double vel = acceleration * time;
    double linearVelocity = driveLeft.getEncoder().getVelocity() * Math.signum(talon.get()); // Get from Sparks
    double velAngle = ypr[0]; // Yaw from pigeon (some more math)
    double crossVelocity = linearVelocity * Math.cos(velAngle);
    double offset = Math.atan(crossVelocity / 80);

    double tickToInch = 1;

    SmartDashboard.putNumber("NEO Position", driveLeft.getEncoder().getPosition() / tickToInch);
    SmartDashboard.putNumber("NEO Vel", driveLeft.getEncoder().getVelocity() / tickToInch);

    if (Gamepad0.getRawButton(3)) {
      driveLeft.getEncoder().setPosition(0);
    }

    if (v == 1) // If you see a target
    {
      if (!Gamepad1.getRawButton(1)) // If x isn't pressed
      {
        double heading_error = -x + offset; // in order to change the target offset (in degrees), add it here
        // How much the limelight is looking away from the target (in degrees)

        double steering_adjust = pid.calculate(heading_error);
        // Returns the next output of the PID controller (where it thinks the turret should go)
        
        double xDiff = 0 - steering_adjust;
        double xCorrect = 0.05 * xDiff;
        talon.set(xCorrect);
        SmartDashboard.putNumber("xCorrect", xCorrect);
      }
    } else if ((v == 0) && (!Gamepad1.getRawButton(1))) // If you do see a target
    {
      if ((currentPos > turretHome) && (currentPos - turretHome > 50)) 
      {
        // If you're to the right of the center, move left until you're within 50 ticks
        talon.set(0.3);
      } else if ((currentPos < turretHome) && (currentPos - turretHome < -50)) 
      {
        talon.set(-0.3);
      }
    }

    // Hard stop configurations
    if (talon.getSelectedSensorPosition() > turretRightStop)
    {
      talon.configPeakOutputReverse(0, 10);
    } else
    {
      talon.configPeakOutputReverse(-1, 10);
    }
    if (talon.getSelectedSensorPosition() < turretLeftStop)
    {
      talon.configPeakOutputForward(0, 10);
    } else
    {
      talon.configPeakOutputForward(1, 10);
    }

    pid.close();
  }
  
  public double degToRad(double deg) {
    return deg * Math.PI / 180;
  }



  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

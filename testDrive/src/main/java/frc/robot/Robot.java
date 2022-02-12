// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODOs
* Add reverse to feed mechanism -
* Add reverse to intake mechanism -
* Implement sensor stop to feed mechanism
* Implement sensor stop to intake mechanism
* Ultrasonic for shooting range
* Color sensor output to LED lightstrips
* change input scaling to 1, 0.7, 0.3 on up, right,down d-pad. -
* LED flash when ball grabbed
* LED signal when balls full
* LED team colors
*/

package frc.robot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private double startTime;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private XboxController xBox = new XboxController(0);
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //private Joystick stickL = new Joystick(0);
  //private Joystick stickR = new Joystick(1);
  private CANSparkMax motorL1 = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax motorL2 = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax motorR1 = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax motorR2 = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax inMotor = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax outMotor = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax feedMotor = new CANSparkMax(8, MotorType.kBrushless);
  private DifferentialDrive driveRobot;
  private MotorControllerGroup leftGroup;
  private MotorControllerGroup rightGroup;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 intakeSensor = new ColorSensorV3(i2cPort);
  private AnalogInput feederSensor = new AnalogInput(0);
  private AnalogInput distanceSensor = new AnalogInput(1);
  //control variables
  private double inputScaling = 0.4;
  private int povState = -1;
  private int speedIndex = 0;
  private double timePassed;
  private double feedStart;
  //configuration variables
  private double inSpeed = -0.7;
  //private double outSpeed = 0.7;
  private double outSpeeds[] = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  private double feedSpeed = 0.8;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    //motorR2.setInverted(true);
    leftGroup = new MotorControllerGroup(motorL1,motorL2);
    rightGroup = new MotorControllerGroup(motorR1,motorR2);
    //leftGroup.setInverted(true);
    //rightGroup.setInverted(true);
    driveRobot = new DifferentialDrive(leftGroup,rightGroup);
    SmartDashboard.putString("aButton", "run intake motor");
    SmartDashboard.putString("xButton", "reverse intake motor");
    SmartDashboard.putString("yButton", "run feed motor");
    SmartDashboard.putString("bButton", "emergerencyBrake");
    SmartDashboard.putString("upD-pad", "full speed");
    SmartDashboard.putString("downD-pad", "slow speed");
    SmartDashboard.putString("rightD-pad", "middle speed");
    SmartDashboard.putString("leftTrigger", "brake throttle");
    SmartDashboard.putString("rightBumper", "shoot");
    SmartDashboard.putString("Left stick", "Arcade drive");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("ballFeederSensorValue", feederSensor.getValue());
    SmartDashboard.putNumber("distanceSensorValue", distanceSensor.getValue());
    SmartDashboard.putNumber("Throwing Speed", outSpeeds[speedIndex]);
    //TODO: outputcolorsensor
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    startTime = Timer.getFPGATimestamp(); // get the match start time


  }

  /** This function is called periodically during autonomous. */
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
    
    double timePassed = Timer.getFPGATimestamp() - startTime; 
    if((timePassed > 0) && (timePassed < 1))
    {
      outMotor.set(0.7);
    } else if((timePassed > 1 ) && (timePassed < 8))
    {
      feedMotor.set(feedSpeed);
    } else 
    {
      outMotor.stopMotor();
      feedMotor.stopMotor();
    }
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double motorSpeed = xBox.getLeftY() * inputScaling;
    SmartDashboard.putNumber("motorSpeed", motorSpeed);
    SmartDashboard.putNumber("inputScaling", inputScaling);
    if(xBox.getBButton())
    {
      driveRobot.stopMotor();
    }
    else 
    {
      driveRobot.arcadeDrive((xBox.getLeftX() * inputScaling),(-xBox.getLeftY() * inputScaling));
    }
    if(povState != xBox.getPOV()) 
    {
      povState = xBox.getPOV();
      if(povState == 180) 
      { 
          inputScaling = 0.4;
      }
      else if(povState == 0)
        {
          inputScaling = 0.8;
        }
        else if(povState == 270)
        { 
          inputScaling = 0.6;
        }
    }
        driveRobot.setMaxOutput(1.0 - xBox.getLeftTriggerAxis());
        
    if(xBox.getYButton())
    {
      //inMotor.set(0.3);
      inMotor.set(inSpeed);
      SmartDashboard.putString("Ybutton", "pushed");
    }
    else
    {
      //inMotor.stopMotor();
      inMotor.stopMotor();
      SmartDashboard.putString("Ybutton", "not pushed");
    }
    
    timePassed = Timer.getFPGATimestamp() - startTime; 

    if(xBox.getLeftBumperPressed())
    {
      speedIndex++;
      speedIndex = speedIndex % 6;
    }

    if(xBox.getRightBumperPressed())
    {
      feedStart = Timer.getFPGATimestamp();
    }

    if(xBox.getRightBumper())
    {
      outMotor.set(outSpeeds[speedIndex]);
      if(Timer.getFPGATimestamp() - feedStart > 0.5)
      {
        feedMotor.set(feedSpeed);
      }
      else if(!xBox.getXButton() && !xBox.getYButton())
      {
        feedMotor.stopMotor();
      }
    }
    else
    {
      outMotor.stopMotor();
    }
    
     if(false || xBox.getAButton())
        {
          feedMotor.set(feedSpeed);
          SmartDashboard.putNumber("feedSpeed", feedSpeed);
        }
    else if(!xBox.getRightBumper())
        {
          feedMotor.stopMotor();
          SmartDashboard.putNumber("feedSpeed", 0);
        }
        
        if(false || (xBox.getXButton() && !xBox.getRightBumper()))
        {
          feedMotor.set(-feedSpeed);
          SmartDashboard.putNumber("feedSpeed", -feedSpeed);
          inMotor.set(-inSpeed);
          SmartDashboard.putNumber("inSpeed", -inSpeed);
        }
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double currentDistanceInches = distanceSensor.getValue() * voltage_scale_factor * 0.0492;
        SmartDashboard.putNumber("Distance Sensor Inches", currentDistanceInches);
        /*else
        {
          feedMotor.stopMotor();
         /SmartDashboard.putNumber("feedSpeed", 0); */
  }


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODOs
!TURN FOR TARGET ACQUISITION
!TOGGLE DIRECTION OF TURN FOR TARGETING
*SPIN UP AND SHOOT WHEN IN RANGE
*USE ENCODERS TO DETERMINE WHEN TO FEED?
*PUT CONFIGURATION FOR LIMELIGHT SEEKING ON SMART DASHBOARD
*/

package frc.robot;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// import javax.management.relation.Relation;

import com.revrobotics.spark.SparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Robot extends TimedRobot {
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private String m_autoSelected;
  private XboxController xBox = new XboxController(0);
  private Joystick joystick = new Joystick(1);
  //private final SendableChooser<String> reelChooser = new SendableChooser<>();
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> limelightchooser = new SendableChooser<>();
  private final String left = "left";
  private final String right = "right";
  //private Joystick stickL = new Joystick(0);
  //private Joystick stickR = new Joystick(1);
  private SparkMax motorL1 = new SparkMax(10, MotorType.kBrushless);
  private SparkMax motorL2 = new SparkMax(11, MotorType.kBrushless);
  private SparkMax motorR1 = new SparkMax(12, MotorType.kBrushless);
  private SparkMax motorR2 = new SparkMax(13, MotorType.kBrushless);
  private SparkMax inMotor = new SparkMax(7, MotorType.kBrushless);
  private SparkMax outMotor = new SparkMax(9, MotorType.kBrushless);
  private SparkMax feedMotor = new SparkMax(8, MotorType.kBrushless);
  private SparkMax reelMotor = new SparkMax(6, MotorType.kBrushless);
  private boolean reelCoast = false;
  private boolean startReleased = false;
  private DifferentialDrive driveRobot;
  //private MotorControllerGroup leftGroup;
  //private MotorControllerGroup rightGroup;
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private ColorSensorV3 ColorSensor = new ColorSensorV3(i2cPort);
  private AnalogInput feederSensor = new AnalogInput(2);
  private AnalogInput preFeedSensor = new AnalogInput(3);
//   private AnalogInput distanceSensor = new AnalogInput(2);
  private SlewRateLimiter driveAccLimiter = new SlewRateLimiter(3);
  // private double heading;
  private Spark ledStrip = new Spark(0);
  private double green = 0.71;
  //private double teamColor;
  private boolean isRedAlliance = false;
  NetworkTable FMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
  //auto variable programs
  //control variables
  private double inputScaling = 0.4;
  private int povState = -1;
  private double oldSlew = 3.0;
  //private int speedIndex = 0;
  private boolean aToggleState = false, bToggleState = false;
  private double highFeedStart, lowFeedStart;
  private boolean feedFlag = false;
  private double reverseDelay;
  private double autoFeedTimeStart;

  //configuration variables
  private double inSpeed = -0.7;
  private double feedSpeed = 0.6;
  private double highSpeed = 0.9;
  private double lowSpeed = 0.45; 

  //Custom Functions

  public double getTeamColor(){    
    isRedAlliance = FMS.getEntry("IsRedAlliance").getBoolean(false);
    if(isRedAlliance)
    {
      return 0.61;
    }
    else
    {
      return 0.87;
    }    
  }

  private void autoFeedRoutine()
  {
    if(feederSensor.getValue()>= 300 || (preFeedSensor.getValue() < 800 && (Timer.getFPGATimestamp() - autoFeedTimeStart) >= 3))
    {
      feedFlag=false;
    }

    if(preFeedSensor.getValue() >= 800 && feederSensor.getValue() < 300 && (Timer.getFPGATimestamp() - reverseDelay > 2) && !xBox.getXButton())
    {
      autoFeedTimeStart = Timer.getFPGATimestamp();
      feedFlag = true;
    }

    if(feedFlag && !(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getXButton() || xBox.getYButton()))
    {
      feedMotor.set(0.3);
    } 
    else if(!feedFlag && !(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getXButton() || xBox.getYButton()))
    {
      feedMotor.stopMotor();
    }
  }

  private void intakeReel()
  {
    double reelRevs = SmartDashboard.getNumber("Reel Revolutions", 9);
    double reelPosition = reelMotor.getEncoder().getPosition();
    if(reelCoast)
    {
     reelMotor.configure(
        new SparkMaxConfig().idleMode(IdleMode.kCoast),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    }
    else if(startReleased)
    {
    //reelMotor.setIdleMode(IdleMode.kBrake);
    reelMotor.configure(
        new SparkMaxConfig().idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters); 
    reelMotor.getEncoder().setPosition(0);
    }
    else if(bToggleState && reelPosition < 0.75 * reelRevs)
    {
      reelMotor.set(0.3);
    }
    else if(bToggleState && reelPosition < reelRevs)
    {
      reelMotor.set(0.2);
    }
    else if(bToggleState)
    {
      reelMotor.stopMotor();
    }
    else if(!bToggleState && reelPosition > 0.25 * reelRevs)
    {
      reelMotor.set(-0.3);
    }
    else if(!bToggleState && reelPosition > 0)
    {
      reelMotor.set(-0.2);
    }
    else if(!bToggleState)
    {
      reelMotor.stopMotor();
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    gyro.calibrate();
    SmartDashboard.putNumber("Reel Revolutions", 9);
    SmartDashboard.putNumber("Slew Rate", 3.0);

    limelightchooser.setDefaultOption(left, left);
    limelightchooser.addOption(right, right);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData("Targeting Turn Direction", limelightchooser);
    // new motorgroups as leader/follower since MotorControllerGroup was depricated 
    SparkBaseConfig l2Config = new SparkMaxConfig().follow(motorL1, /*invert*/ false);
    SparkBaseConfig r2Config = new SparkMaxConfig().follow(motorR1, /*invert*/ false);
    motorL2.configure(l2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorR2.configure(r2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveRobot = new DifferentialDrive(motorL1::set,motorR1::set);
    ledStrip.set(getTeamColor());

  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.updateValues();
    SmartDashboard.putNumber("Ball Sensor", feederSensor.getValue());
    SmartDashboard.putNumber("Gyro Values", gyro.getAngle());
    SmartDashboard.putNumber("Reel Motor Revs", reelMotor.getEncoder().getPosition());
    if(SmartDashboard.getNumber("Slew Rate", 3) != oldSlew)
    {
      driveAccLimiter = new SlewRateLimiter(SmartDashboard.getNumber("Slew Rate", 3));
      oldSlew = SmartDashboard.getNumber("Slew Rate", 3);
    }

  }

  @Override
  public void autonomousInit() {
    gyro.reset();
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    bToggleState = false;
    reelMotor.getEncoder().setPosition(SmartDashboard.getNumber("Reel Revolutions", 9));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }
    

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    gyro.reset();
    // Set motors to brake mode
    motorL1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorL2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    reelMotor.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }
//#endregion
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {

    if(xBox.getBButtonPressed())
    {
      bToggleState = !bToggleState;
    }
    reelCoast = xBox.getStartButton();
    startReleased = xBox.getStartButtonReleased();
    intakeReel();
    if(feederSensor.getValue() > 300 && preFeedSensor.getValue() > 500)
    {
      ledStrip.set(green);
    }
    else
    {
      ledStrip.set(getTeamColor());
    }

    //double motorSpeed = xBox.getLeftY() * inputScaling;
    //SmartDashboard.putNumber("motorSpeed", motorSpeed);
    SmartDashboard.putNumber("Speed of Drivetrain", inputScaling);

      driveRobot.arcadeDrive((xBox.getLeftX() * inputScaling),driveAccLimiter.calculate(-xBox.getLeftY() * inputScaling));
    
    if (povState != xBox.getPOV()) {
      povState = xBox.getPOV();
      switch (povState) {
          case 180 -> inputScaling = 0.4;
          case 0   -> inputScaling = 0.7;
          case 270 -> inputScaling = 0.55;
          case 90  -> inputScaling = 1.0;
      }
  
    }
        
    driveRobot.setMaxOutput(1.0 - xBox.getLeftTriggerAxis());

    //Intake motor intake toggle
    if(xBox.getAButton() || joystick.getRawButton(3))
    {
      aToggleState = true;
    }
    else
    {
      aToggleState = false;
    }
    
    //Intake motor sensor toggle off if both sensors detect ball as there will be 2 balls.
    if(feederSensor.getValue() > 300 && preFeedSensor.getValue() > 800)
    {
      aToggleState = false;
    }

    //Intake motor speed set
    if(aToggleState || joystick.getRawButton(3))
    {
      //inMotor.set(0.3);
      inMotor.set(inSpeed);
      //SmartDashboard.putString("Abutton", "pushed");
    }
    else if(!xBox.getXButton())
    {
      inMotor.stopMotor();
      //SmartDashboard.putString("Abutton", "not pushed");
    }
    
    //timePassed = Timer.getFPGATimestamp() - startTime; 

    /*if(xBox.getLeftBumperPressed())
    {
      speedIndex++;
      speedIndex = speedIndex % outSpeeds.length;
    }*/
    if(xBox.getLeftBumperButtonPressed() || joystick.getRawButtonPressed(1))
    {
      lowFeedStart = Timer.getFPGATimestamp();
    }

    if(xBox.getRightBumperButtonPressed() || joystick.getRawButton(2))
    {
      highFeedStart = Timer.getFPGATimestamp();
    }
    
    if(xBox.getLeftBumperButton() || joystick.getRawButton(1))
    {

      outMotor.set(lowSpeed);
      if(Timer.getFPGATimestamp() - lowFeedStart > 0.5)
      {
        feedMotor.set(feedSpeed);
      }
      else if(!xBox.getXButton() && !xBox.getYButton())
      {
        feedMotor.stopMotor();
      }
    }
    else if(xBox.getRightBumperButton() || joystick.getRawButton(2))
    {
      outMotor.set(highSpeed);
      if(Timer.getFPGATimestamp() - highFeedStart > 1)
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
    
    if(xBox.getYButton())
    {
      feedMotor.set(feedSpeed);
      //SmartDashboard.putNumber("feedSpeed", feedSpeed);
    }
    else if(!(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getXButton()))
    {
      feedMotor.stopMotor();
      //SmartDashboard.putNumber("feedSpeed", 0);
    }

    if((xBox.getXButton() || joystick.getRawButton(4)) && !(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getYButton()))
    {
      feedMotor.set(-1.5 * feedSpeed);
      //SmartDashboard.putNumber("feedSpeed", -feedSpeed);
      inMotor.set(-inSpeed);
      //SmartDashboard.putNumber("inSpeed", -inSpeed);
      aToggleState = false;
    }
    else if(!xBox.getLeftBumperButton() && !(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getYButton()))
    {
      feedMotor.stopMotor();
      //SmartDashboard.putNumber("feedSpeed", 0); 
      if(!aToggleState)
      {
        inMotor.stopMotor();
        //SmartDashboard.putNumber("inSpeed", 0);
      }
    }

    if(xBox.getXButtonReleased())
    {
      reverseDelay = Timer.getFPGATimestamp();
    }
    
    //If a ball is detected at the upper sensor or the lower sensor is clear and the timer was running for more than 3 seconds
    autoFeedRoutine();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {
    // Set the motors to coast for easy robot manipulaton
    motorL1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorL2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    reelMotor.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() 
  {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic()
  {

  }
}

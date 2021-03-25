
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Servo;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.Scanner;
import java.util.Locale;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_driverController;
  private XboxController m_copilotController;


  
  private double cam_y[] = {-12.617279052734375, -11.228164672851562, -7.258064270019531, -3.129486083984375, 2.57806396484375, 10.77984619140625, 24.75890350341797};
  private double cam_angles[] = {41.85999056696892, 63.56852865219116, 63.13391990959644, 61.41732284426689, 54.89308577775955, 53.61325015127659, 33.90286636352539};

  //private double cam_y[] = {-11.2060089111328, -7.54799652099609, -0.55279541015625, 3.23971557617187, 16.1772384643554};
  //private double cam_angles[] = {52.4729946553707, 58.2363437116146, 51.8642348870635, 50.3148376047611, 36.9512795135378};

  private LinearInterpolator interp = new LinearInterpolator();
  private PolynomialSplineFunction polySpline;

  // Drive Base
  private WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(5);
  private WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(6);
  private  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(1);
  private  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(8);

  // Intake
  private  WPI_TalonSRX m_intake = new WPI_TalonSRX(2);

  // Spinner
  private WPI_TalonSRX m_spinner = new WPI_TalonSRX(3);

  //Wheel to load ball

  private WPI_TalonSRX m_wheel = new WPI_TalonSRX(4);

  // Shooter
  private CANSparkMax m_leftShooter = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax m_rightShooter = new CANSparkMax(9, MotorType.kBrushless);

  private CANSparkMax m_turretMotor = new CANSparkMax(10, MotorType.kBrushless);
  private CANPIDController turret_pidController;

  
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;

  private boolean wheelPrev = false; 
  private boolean wheelState = false;

  private LimeLight camera;

  private Servo hoodServo = new Servo(2);

 private Compressor c = new Compressor(0);
  private boolean intakeSwitchPrev = false;
  private boolean intakeState = true;
  private Solenoid intake_in = new Solenoid(2);
  private Solenoid intake_out = new Solenoid(3);

  private double heading;
  private double prev_enc_right;
  private double prev_enc_left;

  private double pos_x;
  private double pos_y;

  private PrintStream autoWriter;
  private Scanner autoScan;
  private boolean prevYPos = false; 
  
  private double target_x = 0;
  private double target_y = 0;
 
  // Kauail Labs AHRS (for heading and rate)
  private AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private double shooterSpeed = -4700;
  private double shooterError = 100;

  private double hoodAngle = 85;
  private double maxSpeed = 1000;

  @Override
  public void robotInit() {
    m_driverController = new XboxController(0);
    m_copilotController = new XboxController(1);

    m_frontRight.configFactoryDefault();
    m_frontLeft.configFactoryDefault();
    m_rearRight.configFactoryDefault();
    m_rearLeft.configFactoryDefault();

    m_rearLeft.setInverted(true);
    m_rearRight.setInverted(true);
    m_frontLeft.setInverted(true);
    m_frontRight.setInverted(true);
    
    m_rearLeft.follow(m_frontLeft);
    m_frontRight.follow(m_rearRight);

    m_pidController = m_leftShooter.getPIDController();
    m_encoder = m_leftShooter.getEncoder();

    turret_pidController = m_turretMotor.getPIDController();

    // set PID coefficients
    m_pidController.setP(0.0005);
    m_pidController.setI(6e-7);
    m_pidController.setD(0);
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-1, 1);

    turret_pidController.setP(0.0001);
    turret_pidController.setI(1e-6);
    turret_pidController.setD(0);
    turret_pidController.setIZone(0);
    turret_pidController.setFF(0);
    turret_pidController.setOutputRange(-1, 1);

    m_rightShooter.follow(m_leftShooter, true);

    c.setClosedLoopControl(true);

    polySpline = interp.interpolate(cam_y, cam_angles);

    m_frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rearRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_frontLeft.configNominalOutputForward(0);
		m_frontLeft.configNominalOutputReverse(0);
		m_frontLeft.configPeakOutputForward(1);
    m_frontLeft.configPeakOutputReverse(-1);
    
    m_rearRight.configNominalOutputForward(0);
		m_rearRight.configNominalOutputReverse(0);
		m_rearRight.configPeakOutputForward(1);
    m_rearRight.configPeakOutputReverse(-1);
    
    m_frontLeft.config_kF(0, 1023.0/maxSpeed);
		m_frontLeft.config_kP(0, 2.0);
		m_frontLeft.config_kI(0, 0.0);
    m_frontLeft.config_kD(0, 10);
    m_frontLeft.configClosedloopRamp(0.1);

    
    m_rearRight.config_kF(0, 1023.0/maxSpeed);
		m_rearRight.config_kP(0, 2.0);
		m_rearRight.config_kI(0, 0.0);
    m_rearRight.config_kD(0, 10);
    m_rearRight.configClosedloopRamp(0.1);
    
    String [] autoModes = {"bouncePath", "barrelRacingPath", "slalomPath", "scratchpad"};
    SmartDashboard.putStringArray("Auto List", autoModes);
  }
  

  @Override
  public void autonomousInit() {
    ahrs.zeroYaw(); 
    prev_enc_left = m_frontLeft.getSelectedSensorPosition();
    prev_enc_right = m_rearRight.getSelectedSensorPosition();
    pos_x = 0;
    pos_y = 0;

    try{        
        String filePath = "/home/lvuser/" + SmartDashboard.getString("Auto Selector", "scratchpad") + ".txt";
        autoScan = new Scanner(new File(filePath));
        autoScan.useLocale(Locale.US);
    }
    catch(FileNotFoundException e)
    {
      System.out.println(e.getCause());
    }    
  }

  @Override
  public void autonomousPeriodic() {
    heading = ahrs.getYaw();

    double enc_right = m_rearRight.getSelectedSensorPosition();
    double enc_left = m_frontLeft.getSelectedSensorPosition();

    double left_right;
    double front_back;

    double diff_enc_left = enc_left - prev_enc_left;
    double diff_enc_right = enc_right - prev_enc_right;

    double diff_enc = ((diff_enc_right - diff_enc_left)/2.0) / 900.0;

    pos_y += diff_enc * Math.cos(heading * Math.PI/180.0);
    pos_x += diff_enc * Math.sin(heading * Math.PI/180.0);

    prev_enc_left = enc_left;
    prev_enc_right = enc_right;

    double x_diff;
    double y_diff;
    double targetHeading;

      x_diff = target_x - pos_x;
      y_diff = target_y - pos_y;

      targetHeading = heading - (Math.atan2(x_diff, y_diff)*180.0/Math.PI);
  
      if(targetHeading > 180)
        targetHeading -= 360;
      if(targetHeading < -180)
        targetHeading += 360;

        if(autoScan.hasNextDouble())
        {
          left_right = autoScan.nextDouble();
          front_back = autoScan.nextDouble();

          targetHeading = autoScan.nextDouble();
          double headingError = targetHeading - heading;
          if(headingError > 180)
          {
            headingError -= 360;
          }
          if(headingError < -180)
          {
            headingError += 360;
          }
        
          left_right += 0.01*headingError;

        }
        else
        {
          left_right = 0;
          front_back = 0;
        }

  
        double left = -front_back - left_right;
        double right = front_back - left_right;

        m_rearRight.set(ControlMode.Velocity, right*maxSpeed);
        m_frontLeft.set(ControlMode.Velocity, left*maxSpeed);
    }

  @Override
  public void teleopPeriodic() {

    double reverse = m_driverController.getTriggerAxis(Hand.kLeft);
    double forward = m_driverController.getTriggerAxis(Hand.kRight);
    double front_back = reverse < 0.01 ? forward : -reverse;
    double left_right = m_driverController.getX(Hand.kLeft);
    

    front_back = front_back > 0 ? Math.pow(Math.abs(front_back), 3) : -Math.pow(Math.abs(front_back), 3);
    left_right = left_right > 0 ? 0.5*Math.pow(Math.abs(left_right), 3) : -0.5*Math.pow(Math.abs(left_right), 3);
    

    double left = -front_back - left_right;
    double right = front_back - left_right;

    m_rearRight.set(ControlMode.Velocity, maxSpeed*right);
    m_frontLeft.set(ControlMode.Velocity, maxSpeed*left);

    if (m_copilotController.getBumperPressed(Hand.kLeft)==true && wheelPrev==false)
    {
      wheelState = !wheelState;
    }
    
    wheelPrev = m_copilotController.getBumperPressed(Hand.kLeft);
    
    if(wheelState == true)
    {
      
      m_pidController.setReference(shooterSpeed, ControlType.kVelocity);

      double turretSpeed = m_copilotController.getX(Hand.kRight);
      double turretAngle = m_copilotController.getY(Hand.kRight);
      boolean camTarget = camera.isTarget();
      double camY = camera.getTy();

      if(Math.abs(turretSpeed) > 0.2 || Math.abs(turretAngle) > 0.2)
      {
        turretSpeed *= 2000;
        hoodAngle -= turretAngle;
      }
      else if(camTarget)
      {
        
        turretSpeed = -100*camera.getTx();
        
        if(camY > cam_y[cam_y.length - 1])
        {
          hoodAngle = cam_angles[cam_y.length - 1];
        }else if(camY < cam_y[0]){
          hoodAngle = cam_angles[0];
        }else{
          hoodAngle = polySpline.value(camY);
        }
      }
      else
      {
        turretSpeed = 0;
      }

      hoodAngle = hoodAngle > 170 ? 170 : hoodAngle < 0 ? 0 : hoodAngle;
      turretSpeed = turretSpeed > 5000 ? 5000 : turretSpeed < -5000 ? -5000 : turretSpeed;
      turret_pidController.setReference(turretSpeed, ControlType.kVelocity);
      hoodServo.setAngle(hoodAngle);
    }
    else
    {
      m_pidController.setReference(0, ControlType.kVelocity);
      hoodServo.setAngle(85);
    }
    
    if(m_copilotController.getXButton()==true && intakeSwitchPrev == false)
    {
        intakeState = !intakeState;
    }
    intakeSwitchPrev = m_copilotController.getXButton();
    intake_in.set(intakeState);
    intake_out.set(!intakeState);

    if(m_copilotController.getAButton())
    {
      m_intake.set(-1);
    }
    else 
    {
      m_intake.set(0);
    }

    if(intakeState == false || wheelState == true)
    {
      m_spinner.set(-0.8);
    }
    else
    {
      m_spinner.set(0);
    }
    if (Math.abs(m_encoder.getVelocity()-shooterSpeed) < shooterError)
    {
      m_wheel.set(-1);
    }
    else
    {
      m_wheel.set(0);
    }

      if (m_copilotController.getYButton() && prevYPos == false){
        ahrs.zeroYaw();
        try{       
          String filePath = "/home/lvuser/" + SmartDashboard.getString("Auto Selector", "scratchpad") + ".txt";
          autoWriter = new PrintStream(filePath);
        }
        catch(FileNotFoundException e)
        {
          System.out.println(e.getCause());
        }    
      }
      if (m_copilotController.getYButton()){
        heading = ahrs.getYaw();
        autoWriter.println(left_right + " " + front_back + " " + heading);
      }
      if (m_copilotController.getYButton() == false && prevYPos == true){
        autoWriter.close();
      }
      prevYPos = m_copilotController.getYButton(); 
    }
  }
 
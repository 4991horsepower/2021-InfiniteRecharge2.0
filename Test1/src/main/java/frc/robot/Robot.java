
package frc.robot;
//package edu.wpi.first.wpilibj.examples.hatchbottraditional.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Servo;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_driverController;
  private XboxController m_copilotController;

  private double cam_y[] = {-16.9921, -13.0028, -11.3832, -8.30721, -4.7067, 0.92128, 9.3866};
  private double cam_angles[] = {62.7953, 32.4803, 49.3701, 49.3701, 50.2362, 50.2362, 47.6378};
  
  private double x_targ[] = {0, -2.5, -5, -5, -5, -5, -2.5, 0, -2.5, -5, -2.5, 0, 0, 0, 0, -2.5, -5};
  private double y_targ[] = {2.5, 5, 7.5, 10, 12.5, 15, 17.5, 20, 22.5, 20, 17.5, 15, 12.5, 10,7.5, 5, 0};

  private LinearInterpolator interp = new LinearInterpolator();
  private PolynomialSplineFunction polySpline;
  private int targetIndex = 0;

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
  private boolean wheelPrev = false; 
  private boolean wheelState = false;

  private LimeLight camera;

  private Servo hoodServo = new Servo(2);

 private Compressor c = new Compressor(0);
  private boolean intakeSwitchPrev = false;
  private boolean intakeState = true;
  private Solenoid intake_in = new Solenoid(2);
  private Solenoid intake_out = new Solenoid(3);
  private boolean motorSwitchPrev = false;
  private boolean motorState = false;

  private double heading;
  private double prev_enc_right;
  private double prev_enc_left;

  private double pos_x;
  private double pos_y;

  // Kauail Labs AHRS (for heading and rate)
  private AHRS ahrs = new AHRS(SPI.Port.kMXP);


  @Override
  public void robotInit() {
    m_driverController = new XboxController(0);
    m_copilotController = new XboxController(1);


    m_rearLeft.setInverted(true);
    m_rearRight.setInverted(true);
    m_frontLeft.setInverted(true);
    m_frontRight.setInverted(true);
    
    // Defaults to drift on neutral, this sets brake mode
    //m_rearLeft.setNeutralMode(NeutralMode.Brake);
    //m_rearRight.setNeutralMode(NeutralMode.Brake);
    //m_frontLeft.setNeutralMode(NeutralMode.Brake);
    //m_frontRight.setNeutralMode(NeutralMode.Brake);


    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    m_rightShooter.follow(m_leftShooter, true);

    c.setClosedLoopControl(true);

    polySpline = interp.interpolate(cam_y, cam_angles);

    m_frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rearRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }
  

  @Override
  public void autonomousInit() {
    ahrs.zeroYaw(); 
    prev_enc_left = m_frontLeft.getSelectedSensorPosition();
    prev_enc_right = m_rearRight.getSelectedSensorPosition();
    pos_x = 0;
    pos_y = 0;    
  }

  @Override
  public void autonomousPeriodic() {
    heading = ahrs.getYaw();

    double enc_left = m_frontLeft.getSelectedSensorPosition();
    double enc_right = m_rearRight.getSelectedSensorPosition();

    double diff_enc_left = enc_left - prev_enc_left;
    double diff_enc_right = enc_right - prev_enc_right;

    double diff_enc = ((diff_enc_right - diff_enc_left)/2.0) / 900.0;

    //System.out.println("Diff enc left: " + diff_enc_left + ", Diff enc right: " + diff_enc_right);

    pos_y += diff_enc * Math.cos(heading * Math.PI/180.0);
    pos_x += diff_enc * Math.sin(heading * Math.PI/180.0);

    //System.out.println("(x, y): " + pos_x + ", " + pos_y);
    prev_enc_left = enc_left;
    prev_enc_right = enc_right;
    double fwd;

    double x_diff;
    double y_diff;
    double targetHeading;
  
     if (targetIndex >= x_targ.length)
    {
      fwd = 0.0;
      x_diff = x_targ[x_targ.length-1] - pos_x;
      y_diff = y_targ[x_targ.length-1] - pos_y;
      targetHeading = heading - (Math.atan2(x_diff, y_diff)*180.0/Math.PI);
    }
    else
    {
      x_diff = x_targ[targetIndex] - pos_x;
      y_diff = y_targ[targetIndex] - pos_y;
      targetHeading = heading - (Math.atan2(x_diff, y_diff)*180.0/Math.PI);
      fwd = 0.3;
    }

    if(targetHeading > 180)
      targetHeading -= 360;
    if(targetHeading < -180)
      targetHeading += 360;
    
    double targetDistance = Math.sqrt(x_diff * x_diff + y_diff * y_diff);
    double targetMinDistance = 0.5;
    if (targetDistance < targetMinDistance)
    {
      targetIndex++;
    }
    System.out.println("Target Distance: " + targetDistance + ", Target Heading: " + (heading - targetHeading) + ", Target Index: " + targetIndex);
  
    double left_right = -targetHeading * 0.005;
    if(left_right > 0.25)
      left_right = 0.25;
    if(left_right < -0.25)
      left_right = -0.25;

    double left = -fwd - left_right;
    double right = fwd - left_right;

    m_frontRight.set(right);
    m_frontLeft.set(left);
  }

  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(m_driverController.getY(Hand.kLeft), m_driverController.getY(Hand.kRight));
    
    double reverse = m_driverController.getTriggerAxis(Hand.kLeft);
    double forward = m_driverController.getTriggerAxis(Hand.kRight);
    double front_back = reverse < 0.1 ? forward : -reverse;
    double left_right = m_driverController.getX(Hand.kLeft);
    

    left_right = left_right > 0 ? 0.5*Math.pow(Math.abs(left_right), 3) : -0.5*Math.pow(Math.abs(left_right), 3);
    

    double left = -front_back - left_right;
    double right = front_back - left_right;

    if(Math.abs(left) < 0.1)
      left = 0;
    if(Math.abs(right) < 0.1)
      right = 0;

    m_frontRight.set(right);
    m_frontLeft.set(left);

    if (m_copilotController.getBumperPressed(Hand.kLeft)==true && wheelPrev==false)
    {

      wheelState = !wheelState;
    }
    
    wheelPrev = m_copilotController.getBumperPressed(Hand.kLeft);
    
    if(wheelState == true)
    {
      m_leftShooter.set(-0.5);
    }
    else
    {
      m_leftShooter.set(0);
    }
    
    //m_rightShooter.set(1);
    
    //m_rightShooter.set(0);

    

    if(m_copilotController.getXButton()==true && intakeSwitchPrev == false)
    {
        intakeState = !intakeState;
    }
    intakeSwitchPrev = m_copilotController.getXButton();
    intake_in.set(intakeState);
    intake_out.set(!intakeState);

    if(intakeState == false)
    {
      m_intake.set(-1);
    }
    else 
    {
      m_intake.set(0);
    }

    if(intakeState == false || wheelState == true)
    {
      m_spinner.set(-1);
    }
    else
    {
      m_spinner.set(0);
    }

    if(m_copilotController.getAButton())
    {
      m_wheel.set(-1);
    }
    else
    {
      m_wheel.set(0);
    }

    double turretSpeed = m_copilotController.getX(Hand.kRight);

    if(Math.abs(turretSpeed) > 0.1)
    {
      m_turretMotor.set(turretSpeed/5);
      System.out.println("Manual Mode");
    }
    else
    {
        boolean camTarget = camera.isTarget();
        double camSpeed;
        double camAngle;
        double hoodAngle;
        if(camTarget == true)
        {
          camSpeed = camera.getTx() * 0.01;
          camAngle = camera.getTy();
          if(camAngle > cam_y[cam_y.length - 1])
          {
            hoodAngle = 0;
          }else if(camAngle < cam_y[0])
          {
            hoodAngle = 170;
          }else{
            hoodAngle = polySpline.value(camAngle);
          }
          hoodServo.setAngle(hoodAngle);
          System.out.println("CamAngle: " + camAngle + ", HoodAnlge: " + hoodAngle);
        }
        else
        {
          System.out.println("No target detected");
          camSpeed = 0;
        }
         
        if(camSpeed > .2)
        {
          camSpeed = .2;
        }
        else if(camSpeed < -.2)
        {
          camSpeed = -.2;
        }
        m_turretMotor.set(-camSpeed);
    }     
  }
  }
 
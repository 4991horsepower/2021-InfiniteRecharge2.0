
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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Scanner;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.Locale;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_driverController;
  private XboxController m_copilotController;

  private double cam_y[] = {-11.2060089111328, -7.54799652099609, -0.55279541015625, 3.23971557617187, 16.1772384643554};
  private double cam_angles[] = {52.4729946553707, 58.2363437116146, 51.8642348870635, 50.3148376047611, 36.9512795135378};

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
  private boolean motorSwitchPrev = false;
  private boolean motorState = false;

  private double heading;
  private double prev_enc_right;
  private double prev_enc_left;

  private double pos_x;
  private double pos_y;

  private PrintStream autoWriter;
  private Scanner autoScan;

  //slalom path 
  //private double x_targets[] = {2.0, 5.0, 5.0, 0.0, -5.0, -5.0, -2.0, 0.0};
  //private double y_targets[] = {3.0, 6.0, 10.0, 15.0, 10.0, 6.0, 3.0, 0.0};

  //barrel racing path
  private double x_targets[] = {0.0,2.5,5.0,2.5,0.0,0.0,-2.5,-5.0,-2.5,0.0,5.0,2.5,0.0,5.0,2.5,0.0,0.0};
  private double y_targets[] = {9.0,11.5,9.0,7.5,9.0,17.0,19.5,17.0,14.5,17.0,22.0,24.5,22.0,0.0};

  //bounce path
  //private double x_targets[] = {0, -5, -5, 0};
  //private double y_targets[] = {10, 0, 10, 0};
  
  private double target_x = 0;
  private double target_y = 0;
 
  // Kauail Labs AHRS (for heading and rate)
  private AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private double shooterSpeed = -4700;
  private double shooterError = 100;

  private double hoodAngle = 85;

  private boolean isRecording = false;
  private String autoPath = "/home/lvuser/barrelRacingPath.txt";
  private double autoTime;

  @Override
  public void robotInit() {
    m_driverController = new XboxController(0);
    m_copilotController = new XboxController(1);

    m_rearLeft.setInverted(true);
    m_rearRight.setInverted(true);
    m_frontLeft.setInverted(true);
    m_frontRight.setInverted(true);
    
    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    m_pidController = m_leftShooter.getPIDController();
    m_encoder = m_leftShooter.getEncoder();

    // PID coefficients
    double kP = 0.0005; 
    double kI = 6e-7;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

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
    autoTime = System.currentTimeMillis() + 500;

    try{
      if (isRecording){
          autoWriter = new PrintStream(autoPath);
      }
      else {
        autoScan = new Scanner(new File(autoPath));
        autoScan.useLocale(Locale.US);
        target_x = autoScan.nextDouble();
        target_y = autoScan.nextDouble();
      }
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

    double diff_enc_left = enc_left - prev_enc_left;
    double diff_enc_right = enc_right - prev_enc_right;

    double diff_enc = ((diff_enc_right - diff_enc_left)/2.0) / 900.0;

    pos_y += diff_enc * Math.cos(heading * Math.PI/180.0);
    pos_x += diff_enc * Math.sin(heading * Math.PI/180.0);

    prev_enc_left = enc_left;
    prev_enc_right = enc_right;

    double fwd = 0.3;

    double x_diff;
    double y_diff;
    double targetHeading;

    if (isRecording){
      if(System.currentTimeMillis() > autoTime){
        autoTime += 500;
        autoWriter.println(pos_x + " " + pos_y);
      }
    }
    else{
      x_diff = target_x - pos_x;
      y_diff = target_y - pos_y;

      targetHeading = heading - (Math.atan2(x_diff, y_diff)*180.0/Math.PI);
  
      if(targetHeading > 180)
        targetHeading -= 360;
      if(targetHeading < -180)
        targetHeading += 360;
  
      if(Math.abs(targetHeading) > 90)
      {
        targetHeading -= 180;
        fwd = -0.3;
        if(targetHeading < -180)
        {
          targetHeading += 360;
        }
      }
      
      double targetDistance = Math.sqrt(x_diff * x_diff + y_diff * y_diff);
      double targetMinDistance = 0.5;
      if (targetDistance < targetMinDistance)
      {
        if(autoScan.hasNextDouble())
        {
          target_x = autoScan.nextDouble();
          target_y = autoScan.nextDouble();
          fwd = 0.3;
        }
        else
        {
          fwd = 0;
        }
      }  
      
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
  }

  @Override
  public void disabledInit() {
    if(isRecording && autoWriter != null)
    {
      autoWriter.close();
    }
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
      m_pidController.setReference(shooterSpeed, ControlType.kVelocity);
    }
    else
    {
      m_pidController.setReference(0, ControlType.kVelocity);
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
      // turned of intake motor for testing purposes
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

    double turretSpeed = m_copilotController.getX(Hand.kRight);

    if(Math.abs(turretSpeed) > 0.1)
    {
      m_turretMotor.set(turretSpeed/5);
    }
    else
    {
        boolean camTarget = camera.isTarget();
        double camSpeed;
        double camAngle;
        //double hoodAngle;
        if(camTarget == true)
        {
          camSpeed = camera.getTx() * 0.01;
          camAngle = camera.getTy();
          /*
          if(camAngle > cam_y[cam_y.length - 1])
          {
            hoodAngle = cam_angles[cam_y.length - 1];
          }else if(camAngle < cam_y[0]){
            hoodAngle = cam_angles[0];
          }else{
            hoodAngle = polySpline.value(camAngle);
          }*/

          if(Math.abs(m_copilotController.getY(Hand.kRight)) > 0.1)
          {
            hoodAngle += m_copilotController.getY(Hand.kRight);
          }

          if(hoodAngle > 170)
            hoodAngle = 170;
          if(hoodAngle < 0)
            hoodAngle = 0;

          //hoodAngle = 85.0*(1.0-m_copilotController.getY(Hand.kRight));
          hoodServo.setAngle(hoodAngle);
          System.out.println("CamAngle: " + camAngle + ", HoodAnlge: " + hoodAngle);
          //System.out.printf("CamAngle: %.4d, HoodAngle: %.4d\n\r", camAngle, hoodAngle);
        }
        else
        {
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
 
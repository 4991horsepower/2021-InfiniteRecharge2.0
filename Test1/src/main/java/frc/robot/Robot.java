/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_gamePad;
  private WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(5);
  private WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(6);
  
  private  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(1);
  private  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(8);
   
  //private  ArcadeDrive m_myRobot = new ArcadeDrive(m_frontLeft, m_frontRight);

  @Override
  public void robotInit() {
   // m_myRobot = new DifferentialDrive(new Talon(0), new Talon(1));
    m_gamePad = new XboxController(0);

    m_rearLeft.setInverted(true);
    m_rearRight.setInverted(true);
    m_frontLeft.setInverted(true);
    m_frontRight.setInverted(true);

    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);
  }

  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(m_gamePad.getY(Hand.kLeft), m_gamePad.getY(Hand.kRight));
    
    double reverse = m_gamePad.getTriggerAxis(Hand.kLeft);
    double front_back = reverse < 0.1 ? m_gamePad.getTriggerAxis(Hand.kRight) : -m_gamePad.getTriggerAxis(Hand.kRight);
    double left_right = m_gamePad.getX(Hand.kLeft);


    left_right = left_right > 0 ? Math.pow(left_right, 2) : -Math.pow(left_right, 2);
    

    double left = -front_back - left_right;
    double right = front_back - left_right;

    if(Math.abs(left) < 0.1)
      left = 0;
    if(Math.abs(right) < 0.1)
      right = 0;

    m_frontRight.set(right);
    m_frontLeft.set(left);
  
  }
}

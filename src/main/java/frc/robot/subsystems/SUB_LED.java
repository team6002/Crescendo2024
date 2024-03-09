// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.utils.Color;
import frc.utils.DuplicateLEDAssignmentException;
import frc.utils.LEDStrip;
import frc.utils.PWMLEDController;
import frc.utils.pattern.ChasePattern;
import frc.utils.pattern.FadePattern;
import frc.utils.pattern.LEDPattern;
import frc.utils.pattern.SolidColorPattern;
import frc.utils.pattern.WavePattern;

public class SUB_LED extends SubsystemBase {
  /** Creates a new SUB_LED. */

  LEDStrip m_strip;
  PWMLEDController m_stripController;
  public SUB_LED() {
    m_strip = new LEDStrip(26, 0);
    m_stripController = new PWMLEDController(9);
    try {
      m_stripController.addStrip(m_strip);
    } catch (DuplicateLEDAssignmentException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  public void setLEDPattern(LEDPattern p_pattern){
    m_strip.setPattern(p_pattern);
  }

  public void setLEDColors(Color p_primary, Color p_secondary){
    m_strip.setPrimaryColor(p_primary);
    m_strip.setSecondaryColor(p_secondary);
  }
  public void setLEDDuration(double p_durations){
      m_strip.setPatternDuration(p_durations);
  }
  public void LEDOn(){
    m_stripController.start();
  }
  public void LEDShootingMode(){
    LEDOn();
    setLEDPattern(new ChasePattern());
    setLEDColors(Color.fromRGB(255, 60, 0), Color.fromRGB(0, 0, 255));
    setLEDDuration(0.75);
  }

  public void LEDAmpMode(){
    LEDOn();
    setLEDPattern(new SolidColorPattern());
    setLEDColors(Color.fromRGB(0, 255, 0), Color.fromRGB(0, 0, 255));
    setLEDDuration(1.5);
  }

  public void LEDIntakeMode(){
    LEDOn();
    setLEDPattern(new FadePattern());
    setLEDColors(Color.fromRGB(0, 0, 255), Color.fromRGB(255, 0, 0));
    setLEDDuration(1.5);
  }
  //make it yellow when no shots
  //make it red when shot is trash
  //make it green when shot is great
  @Override
  public void periodic() {
    m_stripController.updateStrips();
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_LED extends SubsystemBase {
  /** Creates a new SUB_LED. */
  public SUB_LED() {}

  public void setLED(){

  }
  //make it yellow when no shots
  //make it red when shot is trash
  //make it green when shot is great
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

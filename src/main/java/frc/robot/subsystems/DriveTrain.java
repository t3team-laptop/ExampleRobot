// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
 private static TalonSRX motor;
 public DriveTrain(){
  motor = new TalonSRX(Constants.MOTOR_ID);
 }
 public static void setRaw(double value){
  motor.set(ControlMode.PercentOutput, value);
 }
}

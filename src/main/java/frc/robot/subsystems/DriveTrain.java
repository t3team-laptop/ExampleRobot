// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
   private static CANSparkMax LeftMotor;
   private static CANSparkMax RightMotor;
   private static SparkMaxPIDController pidController;

 
   public DriveTrain(){

      LeftMotor = new CANSparkMax(Constants.MOTOR_ID_1, MotorType.kBrushless);
      RightMotor = new CANSparkMax(Constants.MOTOR_ID_2, MotorType.kBrushless);

      LeftEncoderSim = new EncoderSim((Encoder) LeftMotor.getEncoder());
      RightEncoderSim = new EncoderSim((Encoder) RightMotor.getEncoder());
      
      RightMotor.follow(LeftMotor, true);

      pidController = LeftMotor.getPIDController();
      pidController.setP( .01, 0);
      pidController.setI(.001, 0);
      pidController.setD(.1, 0);
      
   }

   public void setOutput(double output){
      LeftMotor.set(output);

   }

   public void setPosition(double position){

      pidController.setReference(position, ControlType.kPosition);

   }

   public void stopMotor(){

      LeftMotor.set(0);

   }

   public boolean isAtSetpoint(double setpoint, double tolerance){
      return Math.abs(setpoint - LeftMotor.getEncoder().getPosition()) < tolerance;
    }


   @Override
    public void periodic(){
     
      SmartDashboard.putNumber("Position Left", LeftMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("Position Right", RightMotor.getEncoder().getPosition());
   
    }

}

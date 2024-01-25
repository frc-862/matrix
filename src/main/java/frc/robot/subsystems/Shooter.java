// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.CAN;
import frc.robot.lib.shuffleboard.LightningShuffleboard;

public class Shooter extends SubsystemBase {
    // CANSparkMax motor1;
    // CANSparkMax motor2;

    // public Shooter() {
    //     motor1 = new CANSparkMax(CAN.SHOOTER_MOTOR1, MotorType.kBrushless);
    //     motor2 = new CANSparkMax(CAN.SHOOTER_MOTOR2, MotorType.kBrushless);

    //     motor1.setInverted(false);
    //     motor2.setInverted(true);
    //     motor1.setIdleMode(IdleMode.kBrake);
    //     motor2.setIdleMode(IdleMode.kBrake);
    // }

    // public double getRPM() {
    //     return (motor1.getEncoder().getVelocity() + motor2.getEncoder().getVelocity()) / 2;
    // }

    // @Override
    // public void periodic() {
    //     LightningShuffleboard.setDouble("shooter", "RPM", getRPM());
    // }

    // public void setPower(double pow) {
    //     motor1.set(pow);
    //     motor2.set(pow);
    // }
    

    TalonFX motor1 = new TalonFX(CAN.SHOOTER_MOTOR1);
    TalonFX motor2 = new TalonFX(CAN.SHOOTER_MOTOR2);

    TalonFXConfiguration config1;
    TalonFXConfiguration config2;

    private final VelocityVoltage rpmPID = new VelocityVoltage(0).withSlot(0);

    
    public Shooter() {
        config1 = new TalonFXConfiguration();
        config2 = new TalonFXConfiguration();

        config1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config1.Slot0.kP = 0;
        config1.Slot0.kI = 0;
        config1.Slot0.kD = 0;
        config1.Slot0.kS = 0;
        config1.Slot0.kV = 0;

        config1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor1.getConfigurator().apply(config1);
        motor2.getConfigurator().apply(config2);
    }

    public void setPower(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("shooter", "RPM", getRPM());
        setRPM(LightningShuffleboard.getDouble("shooter", "target", 0));

        double womp1 = LightningShuffleboard.getDouble("Shooter", "P", 0);
        double womp2 = LightningShuffleboard.getDouble("Shooter", "D", 0);
        double womp3 = LightningShuffleboard.getDouble("Shooter", "S", 0);
        double womp4 = LightningShuffleboard.getDouble("Shooter", "V", 0);

        config1.Slot0.kP = womp1;
        config1.Slot0.kD = womp2;
        config1.Slot0.kS = womp3;
        config1.Slot0.kV = womp4;

        config2.Slot0.kP = womp1;
        config2.Slot0.kD = womp2;
        config2.Slot0.kS = womp3;
        config2.Slot0.kV = womp4;
        
        motor1.getConfigurator().apply(config1);
        motor2.getConfigurator().apply(config2);

    }

    public double getRPM() {
        return (motor1.getVelocity().getValueAsDouble() + motor2.getVelocity().getValueAsDouble()) / 2;
    }

    public void setRPM(double rpm) {
        motor1.setControl(rpmPID.withVelocity(rpm));
    }

    public void stop() {
        setPower(0);
    }
}

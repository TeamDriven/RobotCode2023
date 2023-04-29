// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.MotionMagicConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionMagicLibrary;

public class LED extends SubsystemBase {

  // public AddressableLED m_led = new AddressableLED(23);
  // public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

  private PowerDistribution m_pdp;
  private PneumaticHub LEDController = new PneumaticHub(22);
  private Solenoid stripOneRed = LEDController.makeSolenoid(0);
  private Solenoid stripOneBlue = LEDController.makeSolenoid(1);
  private Solenoid stripOneGreen = LEDController.makeSolenoid(2);
  private Solenoid stripTwoRed = LEDController.makeSolenoid(15);
  private Solenoid stripTwoBlue = LEDController.makeSolenoid(14);
  private Solenoid stripTwoGreen = LEDController.makeSolenoid(13);

  // public DigitalInput testInput = new lInput(4);

  public LED(PowerDistribution pdp) {
    m_pdp = pdp;
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
  }

  public void setYellow() {
    stripOneRed.set(true);
    stripTwoRed.set(true);

    stripOneGreen.set(true);
    stripTwoGreen.set(true);

    stripOneBlue.set(false);
    stripTwoBlue.set(false);
  }

  public void setPurple() {
    stripOneRed.set(true);
    stripTwoRed.set(true);

    stripOneGreen.set(false);
    stripTwoGreen.set(false);

    stripOneBlue.set(true);
    stripTwoBlue.set(true);
  }

  public void setGreen() {
    stripOneRed.set(false);
    stripTwoRed.set(false);

    stripOneGreen.set(true);
    stripTwoGreen.set(true);

    stripOneBlue.set(false);
    stripTwoBlue.set(false);
  }

  public void turnOnRed() {
    stripOneRed.set(true);
    stripTwoRed.set(true);
  }

  public void turnOnBlue() {
    stripOneBlue.set(true);
    stripTwoBlue.set(true);
  }

  public void turnOnGreen() {
    stripOneGreen.set(true);
    stripTwoGreen.set(true);
  }

  public void turnOff() {
    stripOneRed.set(false);
    stripTwoRed.set(false);

    stripOneBlue.set(false);
    stripTwoBlue.set(false);

    stripOneGreen.set(false);
    stripTwoGreen.set(false);
    // m_pdp.setSwitchableChannel(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(getCurrentPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}

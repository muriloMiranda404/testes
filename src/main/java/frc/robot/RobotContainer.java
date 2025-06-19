// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.AlingToTarget;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.Teleop;
import frc.robot.commands.Virar45;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

  // Aqui iniciamos o swerve
  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  Limelight limelight = new Limelight();
  
  // Controle de Xbox, troque para o qual sua equipe estará utilizando
  public XboxController controleXbox = new XboxController(Controle.xboxControle);
  public RobotContainer() {

    // Definimos o comando padrão como a tração
   /*  swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
      () -> MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND),
      () ->  MathUtil.applyDeadband(controleXbox.getRightX(), Constants.Controle.DEADBAND))
    );*/

    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(escolher_modo(1), Constants.Controle.DEADBAND),
      () -> MathUtil.applyDeadband(escolher_modo(2), Constants.Controle.DEADBAND),
      () ->  MathUtil.applyDeadband(escolher_modo(3), Constants.Controle.DEADBAND))
    );
    
    NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));

    // Configure the trigger bindings
    configureBindings();
  }

  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {
    new JoystickButton(controleXbox, 1).onTrue(new Virar45(swerve, 45, this));
    new JoystickButton(controleXbox, 2).onTrue(new Virar45(swerve, -45, this)); 
    new JoystickButton(controleXbox, 4).onTrue(new Virar45(swerve, 5000, this));
    new JoystickButton(controleXbox, 10).onTrue(new ResetPigeon(new Pigeon2(9), swerve));
    new POVButton(controleXbox, 0).whileTrue(new AlingToTarget(limelight, swerve, null, 0, 0));
    new POVButton(controleXbox, 270).whileTrue(new AlingToTarget(limelight, swerve, null, 0, 0));
    new POVButton(controleXbox, 180).whileTrue(new AlingToTarget(limelight, swerve, null, 0, 0));
  }
  // Função que retorna o autônomo
  public Command getAutonomousCommand() {
    // Aqui retornamos o comando que está no selecionador
    return swerve.getAutonomousCommand(Trajetoria.NOME_TRAJETORIA, true);
  }

  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
  public double escolher_modo(int escolha){

    double marcha = 0.5;
    if(controleXbox.getRightBumperButton() && !controleXbox.getLeftBumperButton()){
      marcha = 1.0;
    } else if(controleXbox.getLeftBumperButton() && !controleXbox.getRightBumperButton()){
      marcha = 0.2;
    } else {
      marcha = 0.5;
    }

    double inverter = 1;
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      inverter = -1;
    } else if(DriverStation.getAlliance().get() == Alliance.Red){
      inverter = 1;
    }
    if(escolha == 1){
      return controleXbox.getLeftY() * inverter * marcha;
    } else if(escolha == 2){
      return controleXbox.getLeftX() * inverter * marcha;
    } else if(escolha == 3){
      return controleXbox.getRightX() * marcha;
    }
    return escolha;
  }
}

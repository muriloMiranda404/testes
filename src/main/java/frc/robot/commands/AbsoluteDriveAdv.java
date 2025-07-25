// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.Dimensoes;
import frc.robot.Constants.DriveConstants.Tracao;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * A more advanced Swerve Control System that has 4 buttons for which direction to face
 */
public class AbsoluteDriveAdv extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier headingAdjust;
  private boolean initRotation = false;
  private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. Heading Adjust changes the current heading after being
   * multipied by a constant. The look booleans are shortcuts to get the robot to face a certian direction.
   * Based off of ideas in https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
   *
   * @param swerve            The swerve drivebase subsystem.
   * @param vX                DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY                DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1
   *                          to 1 with deadband already accounted for.  Positive Y is towards the left wall when
   *                          looking through the driver station glass.
   * @param headingAdjust     DoubleSupplier that supplies the component of the robot's heading angle that should be adjusted.
   *                          Should range from -1 to 1 with deadband already accounted for.
   * @param lookAway          Face the robot towards the opposing alliance's wall in the same direction the driver is facing
   * @param lookTowards       Face the robot towards the driver
   * @param lookLeft          Face the robot left
   * @param lookRight         Face the robot right
   */
  public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                                                   BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft, BooleanSupplier lookRight)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingAdjust = headingAdjust;
    this.lookAway = lookAway;
    this.lookTowards = lookTowards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double headingX = 0;
    double headingY = 0;
    Rotation2d newHeading = Rotation2d.fromRadians(0);
    
    // These are written to allow combinations for 45 angles
    // Face Away from Drivers
    if(lookAway.getAsBoolean()){
      headingX = 1;
    }
    // Face Right
    if(lookRight.getAsBoolean()){
      headingY = 1;
    }
    // Face Left
    if(lookLeft.getAsBoolean()){
      headingY = -1;
    }
    // Face Towards the Drivers
    if(lookTowards.getAsBoolean()){
      headingX = -1;
    }

    //Dont overwrite a button press
    if(headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0){
      newHeading = Rotation2d.fromRadians(Tracao.TURN_CONSTANT * -headingAdjust.getAsDouble())
                                                                      .plus(swerve.getHeading());
      headingX = newHeading.getSin();
      headingY = newHeading.getCos();
    }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                         headingX,
                                                         headingY);

    // Prevent Movement After Auto
    if(initRotation)
    {
      if(headingX == 0 && headingY == 0)
      {
        // Get the curretHeading
        Rotation2d firstLoopHeading = swerve.getHeading();

        // Set the Current Heading to the desired Heading
        desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
      }
      //Dont Init Rotation Again
      initRotation = false;
    }

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

    // Caso essa função seja verdadeira a aceleração do robô será limitada
    if(Tracao.accelCorrection) {
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                              Dimensoes.LOOP_TIME, Dimensoes.ROBOT_MASS, 
                                               List.of(Dimensoes.CHASSIS),
                                               swerve.getSwerveDriveConfiguration());

        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());
    }
                                      


    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, Tracao.fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}

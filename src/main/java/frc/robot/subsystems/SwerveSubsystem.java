// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Tracao;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import org.photonvision.targeting.PhotonPipelineResult;

/** 
 * Classe de subsistema onde fazemos a ponte do nosso código para YAGSL
 */
public class SwerveSubsystem extends SubsystemBase {
    // Objeto global da SwerveDrive (Classe YAGSL)
    public SwerveDrive swerveDrive;
    boolean orientation = true;
    Pigeon2 pigeon = new Pigeon2(9);

    private final PhotonCamera photo = new PhotonCamera("limelight"); 
    private AprilTagFieldLayout aprilTagFieldLayout;
    private final Transform3d robotToCam = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5), // Ajuste estas coordenadas para a posição real da sua câmera
        new Rotation3d(0, 0, 0)); // Ajuste estes ângulos para a orientação real da sua câmera
    // Substitua a declaração e inicialização do estimator por:

    private PhotonPoseEstimator estimator;
    
    // Método construtor da classe
    public SwerveSubsystem(File directory) {
        // Seta a telemetria como nível mais alto
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile);
              
                estimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCam);
                
        } catch (Exception e) {
            throw new RuntimeException("Erro ao carregar o layout de AprilTags", e);
        }
    
        // Acessa os arquivos do diretório .JSON
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Dimensoes.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setChassisDiscretization(true, 0.2);
        
        setupPathPlanner();
    }
    
    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        // Atualiza a estimativa de pose com a visão
        Optional<EstimatedRobotPose> result = estimator.update(PhotonPipelineResult.class.cast(robotToCam));
    
        if(result.isPresent()){
            EstimatedRobotPose pose = result.get();
            swerveDrive.addVisionMeasurement(
                pose.estimatedPose.toPose2d(), 
                pose.timestampSeconds
            );
        }
    }

      public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(0.01, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.05, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  //Movimenta o robô com o joystick esquerdo, e mira o robo no ângulo no qual o joystick está apontando
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 1); 
      double yInput = Math.pow(translationY.getAsDouble(), 1); 
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                          headingX.getAsDouble(),
                                                                          headingY.getAsDouble(),
                                                                          getHeading().getRadians(),
                                                                          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  //Movimenta o robô com o joystick esquerdo, e gira o robô na intensidade na qual o joystick direito está para o lado
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 1); 
      double yInput = Math.pow(translationY.getAsDouble(), 1); 
      // Faz o robô se mover
      swerveDrive.driveFieldOriented(new ChassisSpeeds(xInput*swerveDrive.getMaximumChassisVelocity(),
                                                      yInput*swerveDrive.getMaximumChassisVelocity(),
                                                      angularRotationX.getAsDouble()*swerveDrive.getMaximumChassisAngularVelocity()));
    });
    
  }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        
        swerveDrive.driveFieldOriented(speeds);
    }

    // Função drive que chamamos em nossa classe de comando Teleoperado
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) 
    {
      swerveDrive.drive(translation, rotation, fieldRelative, true);
    }

    // Função para obter a velocidade desejada a partir dos inputs do gamepad
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
      return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, 
      getHeading().getRadians());

    }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) 
  {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 0, 0, 0, Tracao.MAX_SPEED);
  }

  // Função que retorna a posição do robô (translação e ângulo), (Usado no autônomo)
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
  
  // Retorna a velocidade relativa ao campo
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  // Retorna a configuração do swerve
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Retorna o objeto de controle, o qual é usado para acessar as velocidades máximas por exemplo
  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(swerveDrive.getGyro().getRotation3d().getAngle(), 360));
  }
  

  // Reseta a odometria para uma posição indicada (Usado no autônomo)
  public void resetOdometry(Pose2d posicao) {
    swerveDrive.resetOdometry(posicao);
  }

  // Seta a velocidade do chassi (Usado no autônomo)
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

    public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
  {
    
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
}

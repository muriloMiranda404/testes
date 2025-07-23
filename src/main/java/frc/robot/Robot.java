// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Constants.CanConstants;

public class Robot extends TimedRobot{
  
  
//motores
public static SparkMax motor_intake = new SparkMax(CanConstants.CANIDS.INTAKE_MOTOR, SparkMax.MotorType.kBrushless);
public static SparkMax motor_da_bola = new SparkMax(CanConstants.CANIDS.ALGAE_MOTOR, SparkMax.MotorType.kBrushless);
public static SparkMax Elevador14 = new SparkMax(CanConstants.CANIDS.ELEVADOR14, SparkMax.MotorType.kBrushless);
public static SparkMax Elevador15 = new SparkMax(CanConstants.CANIDS.ELEVADOR15, SparkMax.MotorType.kBrushless);

//digital input
public static DutyCycleEncoder encoder_intake = new DutyCycleEncoder(CanConstants.DigitalInput.ENCODER_INTAKE);
public static Encoder encoderElev = new Encoder(CanConstants.DigitalInput.ELEVADOR_ENCODER_A, CanConstants.DigitalInput.ELEVADOR_ENCODER_B);
public static DigitalInput fimDeCurso_baixo = new DigitalInput(CanConstants.DigitalInput.FIM_DE_CURSO_BAIXO);
public static DigitalInput fimDeCurso_Cima = new DigitalInput(CanConstants.DigitalInput.FIM_DE_CURSO_ALTO);
public static DigitalInput fim_de_curso_Coral = new DigitalInput(CanConstants.DigitalInput.FIM_DE_CURSO_INTAKE);

//PIDs e variaveis 
public static double Kp_intake = 0.01;
public static double Ki_intake = 0;
public static double Kd_intake= 0;
public static double setpoint_intake = 60.0;
public static boolean trava_motor_bola=false;
public static boolean tem_bola_intake=false;
public static double speed_intake;
public static double angulo_intake;
public static double output_intake;

public static double KpElev = 0.01;
public static double KiElev = 0;
public static double KdElev = 0;
public static double tolerancia_elevador = 30.0;
public static double setpoint_elevador;
public static double angulo_elevador;
public static double output_elevador;
public static boolean seguranca_elevador = false;
public static boolean zerar_elevador=false;

public static PIDController PIDElevador = new PIDController(KpElev, KiElev, KdElev);
public static PIDController PID_inatake = new PIDController(Kp_intake, Ki_intake, Kd_intake);

//variaveis de log
public static double lasttick;
public static int botao = 0;

//variaveis para maquina de estado
public static int estado_atual= 0;
public static boolean hablita_maquina_estados = false;
public static double setpoint_1_elevador = 0;
public static double setpoint_1_intake = 0;
public static double setpoint_2_elevador = 0;
public static double setpoint_2_intake = 0;
public static double setpoint_3_elevador = 0;
public static double setpoint_3_intake = 0;


//outros componentes 
public static Joystick joystick = new Joystick(CanConstants.USB.JOYSTICK_INTAKE);
public static Pigeon2 pigeon = new Pigeon2(CanConstants.CANIDS.PIGEON2);
public static Timer timer = new Timer();
private static  Command m_autonomousCommand;
private static RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    //container
    m_robotContainer = new RobotContainer();

    //encoders
    encoderElev.setDistancePerPulse(360.0/2048.0);
    encoderElev.setReverseDirection(true);
    encoder_intake.setDutyCycleRange(0, 360);
   
    //PIDs
    PIDElevador.setTolerance(tolerancia_elevador);
    PID_inatake.setTolerance(4.0);

    //setpoints
    setpoint_elevador = 0.0;
    setpoint_intake = 58.0;

    //outros 
    System.out.println("Robo iniciado\n");
    CameraServer.startAutomaticCapture();
    pigeon.reset();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //logs
    timer.reset();
    timer.start();
    lasttick = 0.0;
    System.out.println("Teleop iniciado\n");

    //encoder e setpoint
    encoderElev.reset();
    setpoint_elevador = 0.0;
    setpoint_intake = 58.0;
  }

  @Override
  public void teleopPeriodic(){
    
    if(hablita_maquina_estados ==false)
    {
      if(joystick.getRawButtonPressed(2)){//L1
        setpoint_1_elevador=setpoint_elevador;
        setpoint_1_intake = 68.0;
        setpoint_2_elevador = 0;
        setpoint_2_intake = 68.0;
        setpoint_3_elevador = 0.0;
        setpoint_3_intake = 55.0;
        hablita_maquina_estados = true;
        estado_atual = 1;

        motor_da_bola.set(0.2);
        trava_motor_bola=true;
      }
      if(joystick.getRawButtonPressed(4)){//L2
        setpoint_1_elevador=0.0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 210.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 210.0;
        setpoint_3_intake = 72.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }
      if(joystick.getRawButtonPressed(1)){//L4

        setpoint_1_elevador=0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 1480.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 1480.0;
        setpoint_3_intake = 92.0;
        hablita_maquina_estados = true;
        estado_atual = 1;

      }
      if(joystick.getRawButtonPressed(2)){//L3

        setpoint_1_elevador=0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 769.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 769.0;
        setpoint_3_intake = 72.0;
        hablita_maquina_estados = true;
        estado_atual = 1;

      }
      if(joystick.getRawButton(8) && !joystick.getRawButton(9)){//ir para L2 da bola
        setpoint_1_elevador=624.0;
        setpoint_1_intake = 225.0;
        setpoint_2_elevador = 624.0;
        setpoint_2_intake = 225.0;
        setpoint_3_elevador = 624.0;
        setpoint_3_intake = 225.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }
      if(!joystick.getRawButton(8) && joystick.getRawButton(9)){//ir para L3 da bola
        setpoint_1_elevador=1107.0;
        setpoint_1_intake = 225.0;
        setpoint_2_elevador = 1107.0;
        setpoint_2_intake = 225.0;
        setpoint_3_elevador = 1107.0;
        setpoint_3_intake = 225.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
    }
    if(joystick.getRawButton(7)){//processador
      setpoint_1_elevador=0.0;
      setpoint_1_intake = 225.0;
      setpoint_2_elevador = 0.0;
      setpoint_2_intake = 225.0;
      setpoint_3_elevador = 0.0;
      setpoint_3_intake = 225.0;
      hablita_maquina_estados = true;
      estado_atual = 1;
  }
    }
    
    
    //////////////////////////////////////calcular e controlar elevador////////////////////////////////////////////////////////////////
    angulo_elevador = encoderElev.getDistance();
    output_elevador = PIDElevador.calculate(angulo_elevador, setpoint_elevador);
  
    //ler fim de curso de cima
    if(fimDeCurso_Cima.get()){
      if(output_elevador>0)output_elevador=0;
      if(setpoint_elevador>1480)setpoint_elevador=1480.0;
    }
    
    //ler fim de curso de baixo
    if(fimDeCurso_baixo.get()){
      if(output_elevador<0){
        output_elevador=0;
      }  
      if(setpoint_elevador<0.0){
        setpoint_elevador=0.0;
      }
      if(zerar_elevador==true&&(angulo_elevador>tolerancia_elevador||angulo_elevador<-1*tolerancia_elevador)){
        zerar_elevador=false;  
      }
    }else {
      zerar_elevador=true;
    }
  
    //segurança do elevador em relação ao intake
    if(seguranca_elevador == true){
      output_elevador=0.0;
    }
    
    Elevador14.set(output_elevador);
    Elevador15.set(-output_elevador );
    //////////////////////////////////////////////// fim do código do elevador //////////////////////////////////////////////////////
  
    //////////////////////////////////////// inicio do codigo do intake ////////////////////////////////////////////////////////////
    angulo_intake = encoder_intake.get() * 360.0;
    output_intake = PID_inatake.calculate(angulo_intake, setpoint_intake);
  
      //ler posição minima do intake
      if(angulo_intake < 55){

        if(output_intake<0){
          output_intake=0.0;
        }
        if(setpoint_intake<55){
          setpoint_intake = 55.0;
        }

      }
  
      //ler posição maxima do intake 
      if(angulo_intake > 230){

        if(output_intake>0){
          output_intake=0.0;
        }
        if(setpoint_intake> 230){
          setpoint_intake = 230.0;
        }

      }
      //saida 
    motor_intake.set(output_intake);
  
    //segurança do elevador 
    if(angulo_intake < 64){
      seguranca_elevador = true;
    } else {
      seguranca_elevador = false;
    }

    //empurrar o coral
    if(joystick.getRawButton(10)){
    motor_da_bola.set(0.3);
    botao++;
    }
    //puxar o coral
    else if(joystick.getRawButton(11)){
    motor_da_bola.set(-0.3);
    botao++;
    }
    //trava para parar 
    if(botao > 2){
    botao = 0;
    motor_da_bola.set(0);
    }
    /////////////////////////////////////////// fim do código do intake //////////////////////////////////////////////////////////
  
    ///////////////////////////////////////inicio do código do  motor da bola e do coral ////////////////////////////////////////////
    
    //liga o motor no L1
    if(trava_motor_bola == true) {
      motor_da_bola.set(0.2);
    }

    if(!fim_de_curso_Coral.get() && trava_motor_bola == true){
      trava_motor_bola=false;
      motor_da_bola.set(0);
    }    
      
    ///////////////////////////////////////fim do código da bola e do coral ////////////////////////////////////////////
    
    ////////////////////////////////// Inicio do código da maquina de estados////////////////////////////////////////
    if(hablita_maquina_estados == true)
    {
      
      switch(estado_atual)
      {
        case 1:
        {
          setpoint_elevador=setpoint_1_elevador;
          setpoint_intake=setpoint_1_intake;
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint()){
            estado_atual = 2;
            setpoint_elevador=setpoint_2_elevador;
            setpoint_intake=setpoint_2_intake;
          }
        }break;
        case 2:
        {
          
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint()){
            estado_atual = 3;
            setpoint_elevador=setpoint_3_elevador;
            setpoint_intake=setpoint_3_intake;
          
          }
        }break;
        case 3:
        {
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint()){
            hablita_maquina_estados = false;
          }
        }break;
      }
    }
    ////////////////////////////////// Fim do código da maquina de estados////////////////////////////////////////
    //imprimir os logs
    if(timer.get() > (lasttick + 2.0))
    {
      lasttick = timer.get();
      System.out.printf("Set_point elevador: %2f\n", setpoint_elevador);
      System.out.printf("Angulo_elevador: %2f\n", angulo_elevador);
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

}
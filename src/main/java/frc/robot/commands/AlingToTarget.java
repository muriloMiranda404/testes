package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class AlingToTarget extends Command {

    private final Limelight limelight;
    private final SwerveSubsystem subsystem;
    private final PIDController rotationController;
    private final PIDController xController;  // Novo controlador para movimento em X
    private final PIDController yController;  // Controlador para distância
    private final RobotContainer container;
    private Translation2d translation;
    
    // Constantes para ajuste fino
    private static final double kP_ROTATION = 0.1;
    private static final double kI_ROTATION = 0.01;
    private static final double kD_ROTATION = 0.005;
    
    private static final double kP_X = 0.1;  // Ajuste esses valores conforme necessário
    private static final double kI_X = 0.01;
    private static final double kD_X = 0.005;
    
    private static final double kP_Y = 0.1;
    private static final double kI_Y = 0.01;
    private static final double kD_Y = 0.005;
    private static final double MAX_VELOCIDADE_Y = 0.3;

    private static final double KP_A = 0.1;
    private static final double KI_A = 0.01;
    private static final double KD_A = 0.005;
    private PIDController aController;
    
    double setpointX;
    double setpointY;
    
    private static final double TOLERANCIA_ALINHAMENTO = 0.6;
    private static final double MAX_CORRECAO = 0.5;
    private static final double MAX_VELOCIDADE_X = 0.3;  // Velocidade máxima no eixo X
    private static final double TIMEOUT_SECONDS = 3.0;
    private static final int MAX_TENTATIVAS = 3;
    private static final double ZONA_MORTA = 0.05;
    private static final double ANGULO_MAXIMO = 30.0;
    
    private final Timer timer = new Timer();
    private int tentativas = 0;
    private boolean ultimoEstadoTarget = false;
    private double ultimaTx = 0.0;
    private double ultimoTempoMudanca = 0.0;
    private static final double TEMPO_MINIMO_ESTAVEL = 0.5;
    
    public AlingToTarget(Limelight limelight, SwerveSubsystem subsystem, RobotContainer container, double setpointX, double setpointY) {
        if (limelight == null || subsystem == null || container == null) {
            throw new IllegalArgumentException("Parâmetros não podem ser nulos");
        }
        this.limelight = limelight;
        this.subsystem = subsystem;
        this.container = container;
        this.aController = new PIDController(KP_A, KI_A, KD_A);
        this.setpointX = setpointX;
        this.setpointY = setpointY;
        this.rotationController = new PIDController(kP_ROTATION, kI_ROTATION, kD_ROTATION);
        this.xController = new PIDController(kP_X, kI_X, kD_X);
        this.yController = new PIDController(kP_Y, kI_Y, kD_Y);
        addRequirements(subsystem, limelight); // Removido limelight dos requirements pois não é um subsistema
    }

    @Override
    public void initialize() {
        try {
            rotationController.reset();
            xController.reset();
            rotationController.setSetpoint(0);
            xController.setSetpoint(0);  // Queremos que a distância em X seja 0
            rotationController.setTolerance(TOLERANCIA_ALINHAMENTO);
            xController.setTolerance(0.1);  // 10cm de tolerância para X
            System.out.println("Iniciando alinhamento com a tag...");
            timer.reset();
            timer.start();
            tentativas = 0;
            ultimoEstadoTarget = false;
            ultimaTx = 0.0;
            ultimoTempoMudanca = 0.0;
        } catch (Exception e) {
            System.err.println("Erro na inicialização do alinhamento: " + e.getMessage());
        }
    }

    @Override
    public void execute() {
        try {
            boolean temTarget = limelight.getHasTarget();
            // Detecta perda de alvo
            if (temTarget != ultimoEstadoTarget) {
                ultimoEstadoTarget = temTarget;
                if (!temTarget) {
                    tentativas++;
                    System.out.println("Tag perdida! Tentativa " + tentativas + " de " + MAX_TENTATIVAS);
                }
            }
            
            if (temTarget) {
                double ta = limelight.getTa();
                double tx = limelight.getTx();
                double ty = limelight.getTy();
                double distancia = limelight.estimateDistance(); // Método que calcula a distância
                limelight.setLedMode(1);
                
                // Proteção contra valores inválidos
                if (Double.isNaN(tx) || Double.isNaN(distancia) || Double.isNaN(ty) || Double.isNaN(ta)) {
                    System.err.println("Valores inválidos recebidos da Limelight");
                    return;
                }
                
                // Verifica mudanças bruscas
                if (Math.abs(tx - ultimaTx) > 10.0) {
                    ultimoTempoMudanca = timer.get();
                }
                ultimaTx = tx;
                
                // Só move se estiver estável por tempo suficiente
                if (timer.get() - ultimoTempoMudanca > TEMPO_MINIMO_ESTAVEL) {
                    double correcaoRotacao = rotationController.calculate(tx, setpointX);
                    double correcaoX = xController.calculate(ty, setpointY);
                    double correcaoY = yController.calculate(distancia, setpointY);
                    
                    // Aplica zona morta
                    if (Math.abs(correcaoRotacao) < ZONA_MORTA) correcaoRotacao = 0;
                    if (Math.abs(correcaoX) < ZONA_MORTA) correcaoX = 0;
                    if (Math.abs(correcaoY) < ZONA_MORTA) correcaoY = 0;

                    // Limita correções
                    correcaoRotacao = Math.min(Math.max(correcaoRotacao, -MAX_CORRECAO), MAX_CORRECAO);
                    correcaoX = Math.min(Math.max(correcaoX, -MAX_VELOCIDADE_X), MAX_VELOCIDADE_X);
                    correcaoY = Math.min(Math.max(correcaoY, -MAX_VELOCIDADE_Y), MAX_VELOCIDADE_Y);
                    
                    // Movimento mais suave em ângulos grandes
                    if (Math.abs(tx) > ANGULO_MAXIMO) {
                        correcaoRotacao *= 0.5;
                        correcaoX *= 0.5;
                        correcaoY *= 0.5;
                    }
                    
                    translation = new Translation2d(-correcaoY * ta, correcaoX);
                    subsystem.drive(translation, -correcaoRotacao, true);
                    
                    // Debug
                    SmartDashboard.putNumber("Tempo de Alinhamento", timer.get());
                    SmartDashboard.putNumber("Tentativas", tentativas);
                    SmartDashboard.putNumber("Tempo desde última mudança", timer.get() - ultimoTempoMudanca);
                    SmartDashboard.putNumber("Distancia", distancia);
                    SmartDashboard.putBoolean("Distancia OK", yController.atSetpoint());
                    limelight.setLedMode(2);
                    
                    System.out.printf("Alinhando - TX: %.2f° | Dist X: %.2f | Rot: %.2f | X: %.2f%n", 
                        tx, distancia, correcaoRotacao, correcaoX);
                        SmartDashboard.putNumber("tx", tx);      
                        SmartDashboard.putBoolean("tag achada", limelight.getHasTarget());
                        SmartDashboard.putNumber("ty", limelight.getTy());
                        SmartDashboard.putBoolean("foi alinhada", rotationController.atSetpoint());
                        SmartDashboard.putBoolean("foi alinhada", xController.atSetpoint());
                       }
            } else {
                subsystem.drive(new Translation2d(), 0, true);
                System.out.println("Nenhuma tag detectada - Verifique o campo de visão da Limelight");
            }
        } catch (Exception e) {
            System.out.println("Erro durante execução do alinhamento: " + e.getMessage());
            subsystem.drive(new Translation2d(), 0, true); // Para o robô com segurança
        }
    }

    @Override
    public boolean isFinished() {
        return (rotationController.atSetpoint() && 
                xController.atSetpoint() && 
                yController.atSetpoint()) ||
               timer.hasElapsed(TIMEOUT_SECONDS) ||
               tentativas >= MAX_TENTATIVAS;
    }

    @Override
    public void end(boolean interrupted) {
        try {
            subsystem.drive(new Translation2d(), 0, true);
            timer.stop();
            String mensagem;
            if (timer.hasElapsed(TIMEOUT_SECONDS)) {
                mensagem = "Alinhamento interrompido por timeout!";
            } else if (tentativas >= MAX_TENTATIVAS) {
                mensagem = "Alinhamento interrompido após " + tentativas + " tentativas!";
            } else if (interrupted) {
                mensagem = "Alinhamento interrompido pelo usuário!";
            } else {
                mensagem = "Alinhamento concluído com sucesso!";
            }
            System.out.println(mensagem);
        } catch (Exception e) {
            System.err.println("Erro ao finalizar alinhamento: " + e.getMessage());
        }
    }
}
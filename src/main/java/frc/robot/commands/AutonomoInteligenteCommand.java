package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Limelight;

public class AutonomoInteligenteCommand extends Command {
    private int estado = 0;
    private final Limelight limelight;
    private int nivelEscolhido = 4; // Começa tentando L4
    
    public AutonomoInteligenteCommand(Limelight limelight) {
        this.limelight = limelight;
    }
    
    @Override
    public void initialize() {
        estado = 0;
        Robot.setpoint_elevador = 0.0;
        Robot.setpoint_intake = 58.0;
        nivelEscolhido = 4;
    }
    
    private boolean verificarNivelOcupado(int nivel) {
        // Aqui você pode implementar a lógica para verificar se tem nota no nível
        // Por exemplo, usando sensores de cor, distância ou câmera
        switch(nivel) {
            case 4:
                return limelight.getHasTarget() && limelight.getTa() > 2.0;
            case 3:
                return limelight.getHasTarget() && limelight.getTa() > 1.5;
            case 2:
                return limelight.getHasTarget() && limelight.getTa() > 1.0;
            default:
                return false;
        }
    }
    
    private double getSetpointParaNivel(int nivel) {
        switch(nivel) {
            case 4:
                return 1480.0; // L4
            case 3:
                return 769.0;  // L3
            case 2:
                return 210.0;  // L2
            default:
                return 0.0;
        }
    }
    
    @Override
    public void execute() {
        switch(estado) {
            case 0: // Verifica nível e decide
                if (verificarNivelOcupado(nivelEscolhido)) {
                    nivelEscolhido--; // Tenta o próximo nível mais baixo
                    if (nivelEscolhido < 2) {
                        nivelEscolhido = 2; // Não desce mais que L2
                    }
                    estado = 0; // Verifica novamente
                } else {
                    Robot.setpoint_elevador = getSetpointParaNivel(nivelEscolhido);
                    estado = 1;
                }
                break;
                
            case 1: // Ajusta intake
                Robot.setpoint_intake = 71.0;
                if (Robot.PID_inatake.atSetpoint()) {
                    estado = 2;
                }
                break;
                
            case 2: // Eleva para posição escolhida
                if (Robot.PIDElevador.atSetpoint()) {
                    estado = 3;
                }
                break;
                
            case 3: // Atira a nota
                Robot.motor_da_bola.set(0.3);
                if (!Robot.fim_de_curso_Coral.get()) {
                    estado = 4;
                }
                break;
                
            case 4: // Finaliza
                Robot.motor_da_bola.set(0);
                Robot.setpoint_elevador = 0.0;
                break;
        }
    }
    
    @Override
    public boolean isFinished() {
        return estado == 4 && Robot.PIDElevador.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        Robot.motor_da_bola.set(0);
        Robot.setpoint_elevador = 0.0;
        Robot.setpoint_intake = 58.0;
    }
}
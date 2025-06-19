package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomoConfig {
    private SendableChooser<String> posicaoInicial;
    private SendableChooser<String> alvo;
    private SendableChooser<String> sequencia;

    public AutonomoConfig() {
        posicaoInicial = new SendableChooser<>();
        alvo = new SendableChooser<>();
        sequencia = new SendableChooser<>();

        // Configurar opções de posição inicial
        posicaoInicial.setDefaultOption("Centro", "CENTRO");
        posicaoInicial.addOption("Esquerda", "ESQUERDA");
        posicaoInicial.addOption("Direita", "DIREITA");

        // Configurar opções de alvo
        alvo.setDefaultOption("Speaker", "SPEAKER");
        alvo.addOption("Amp", "AMP");
        alvo.addOption("Apenas Sair", "SAIR");

        // Configurar sequências
        sequencia.setDefaultOption("Pontuar e Sair", "PONTUAR_SAIR");
        sequencia.addOption("Pontuar e Pegar Nota", "PONTUAR_PEGAR");
        sequencia.addOption("Apenas Sair", "APENAS_SAIR");

        // Adicionar ao Shuffleboard
        SmartDashboard.putData("Posição Inicial", posicaoInicial);
        SmartDashboard.putData("Alvo", alvo);
        SmartDashboard.putData("Sequência", sequencia);
    }

    public String getPosicaoInicial() {
        return posicaoInicial.getSelected();
    }

    public String getAlvo() {
        return alvo.getSelected();
    }

    public String getSequencia() {
        return sequencia.getSelected();
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    private final TalonFXConfiguration configuration;

    public Elevator() {
        elevatorLeader = new TalonFX(15, "*");
        elevatorFollower = new TalonFX(16, "*");

        configuration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(3.4)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.9)
                .withKG(1.06)
                .withKV(0.0)
                .withKA(0.0)
                .withGravityType(GravityTypeValue.Elevator_Static)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(90)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(90)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(8.0)
                .withMotionMagicCruiseVelocity(8.0)
            );
    }
}

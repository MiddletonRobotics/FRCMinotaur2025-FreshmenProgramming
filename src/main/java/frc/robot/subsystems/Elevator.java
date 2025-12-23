package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    private final TalonFXConfiguration configuration;

    private VoltageOut voltageRequest;
    private MotionMagicExpoDutyCycle positionRequest;

    private StatusSignal<Angle> elevatorLeadPosition;
    private StatusSignal<Angle> elevatorFollowerPosition;

    private StatusSignal<AngularVelocity> elevatorLeadVelocity;
    private StatusSignal<AngularVelocity> elevatorFollowerVelocity;

    private StatusSignal<Temperature> elevatorLeadTempurature;
    private StatusSignal<Temperature> elevatorFollowerTempurature;

    private double elevatorSetpoint = 0.0;
    private double elevatorSprocketDiameter = (16 * 0.25) / Math.PI;

    private Alert elevatorLeaderTempuratureFault;

    public Elevator() {
        elevatorLeader = new TalonFX(15, "*");
        elevatorFollower = new TalonFX(16, "*");

        elevatorLeaderTempuratureFault = new Alert("Elevator Leader Tempurature is very high, consider tuning off the robot and letting it cool", AlertType.kWarning);

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
                .withMotionMagicAcceleration(8.0 / (Math.PI * elevatorSprocketDiameter) * 6)
                .withMotionMagicCruiseVelocity(8.0 / (Math.PI * elevatorSprocketDiameter) * 6)
            );

        elevatorLeader.getConfigurator().apply(configuration);
        elevatorFollower.getConfigurator().apply(configuration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

        voltageRequest = new VoltageOut(0.0);
        positionRequest = new MotionMagicExpoDutyCycle(0.0);

        elevatorLeader.setPosition(0.0);
        elevatorFollower.setPosition(0.0);

        elevatorLeadPosition = elevatorLeader.getPosition();
        elevatorFollowerPosition = elevatorFollower.getPosition();
        elevatorLeadVelocity = elevatorLeader.getVelocity();
        elevatorFollowerVelocity = elevatorFollower.getVelocity();
        elevatorLeadTempurature  = elevatorLeader.getDeviceTemp();
        elevatorFollowerTempurature = elevatorFollower.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            200, 
            elevatorLeadPosition,
            elevatorFollowerPosition,
            elevatorLeadVelocity,
            elevatorFollowerVelocity
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            elevatorLeadTempurature,
            elevatorFollowerTempurature
        );
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorLeadPosition,
            elevatorFollowerPosition,
            elevatorLeadVelocity,
            elevatorFollowerVelocity,
            elevatorLeadTempurature,
            elevatorFollowerTempurature
        );

        elevatorLeaderTempuratureFault.set(elevatorLeadTempurature.getValue().in(Celsius) > 85);

        SmartDashboard.putNumber("Elevator Current Position", getPosition());
        SmartDashboard.putNumber("Elevator Setpoint Postion", elevatorSetpoint);
    }

    public double getPosition() {
        return ((elevatorLeadPosition.getValue().in(Rotations) * (1/6) * (Math.PI *elevatorSprocketDiameter) * 3) + (elevatorFollowerPosition.getValue().in(Rotations) * (1/6) * (Math.PI *elevatorSprocketDiameter) * 3)) / 2;
    }

    public void setVoltage(double voltage) {
        elevatorLeader.setControl(voltageRequest.withOutput(voltage));
        elevatorFollower.setControl(voltageRequest.withOutput(0.0));
    }

    public void setMotionMagicSetpoint(double meters) {
        elevatorSetpoint = meters;
        elevatorLeader.setControl(positionRequest.withPosition((meters / 3) / (Math.PI * elevatorSprocketDiameter) * 6));
        elevatorFollower.setControl(positionRequest.withPosition((meters / 3) / (Math.PI * elevatorSprocketDiameter) * 6));
    }
}

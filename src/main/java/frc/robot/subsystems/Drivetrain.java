package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    
    private WPI_TalonSRX[] leftMotors;
    private WPI_TalonSRX[] rightMotors;
    private DifferentialDrive drivetrain;

    public Drivetrain() {
        leftMotors = new WPI_TalonSRX[2];
        leftMotors[0] = new WPI_TalonSRX(1);
        leftMotors[1] = new WPI_TalonSRX(3);

        rightMotors = new WPI_TalonSRX[2];
        rightMotors[0] = new WPI_TalonSRX(2);
        rightMotors[1] = new WPI_TalonSRX(4);

        for(int i = 0; i < 2; i++) {
            leftMotors[i].setInverted(true);
            leftMotors[i].setNeutralMode(NeutralMode.Brake);
            leftMotors[i].configSelectedFeedbackSensor(FeedbackDevice.None);
            leftMotors[i].setSelectedSensorPosition(0);
            leftMotors[i].configVelocityMeasurementWindow(1);
            leftMotors[i].configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);

            rightMotors[i].setInverted(true);
            rightMotors[i].setNeutralMode(NeutralMode.Brake);
            rightMotors[i].configSelectedFeedbackSensor(FeedbackDevice.None);
            rightMotors[i].setSelectedSensorPosition(0);
            rightMotors[i].configVelocityMeasurementWindow(1);
            rightMotors[i].configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);

            if(i != 0) {
                leftMotors[i].follow(leftMotors[0]);
                rightMotors[i].follow(rightMotors[0]);
            }
        }

        drivetrain = new DifferentialDrive(leftMotors[0], rightMotors[0]);
    }

    public void drive(double x, double z) {
        drivetrain.arcadeDrive(x, z);
    }

    public void stop() {
        drivetrain.arcadeDrive(0, 0);
    }

    public void setMaxPower(double maxPower) {
        drivetrain.setMaxOutput(maxPower);
    }
}

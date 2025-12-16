package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopTank;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final Drivetrain drivetrain;

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    
    private SendableChooser<Command> autoChooser;

    private Drivetrain buildDrivetrain() {
        return new Drivetrain();
    }

    public Drivetrain getDrivetrainSubsystem() {
        return drivetrain;
    }

    public RobotContainer() {
        drivetrain = buildDrivetrain();

        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);

        autoChooser = AutoBuilder.buildAutoChooser();

        RobotController.setBrownoutVoltage(6.5);
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(new TeleopTank(
            drivetrain, 
            () -> driverController.getLeftY(), 
            () -> driverController.getLeftX()
        ));
    }

    public Command getAutonomousCommand() {
        return Commands.idle();
    }
}

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TeleopTank extends Command {
    private final Drivetrain drivetrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier rotationSupplier;

    public TeleopTank(Drivetrain drivetrain, DoubleSupplier forwardSupplier, DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(forwardSupplier.getAsDouble(), rotationSupplier.getAsDouble());
    }
}

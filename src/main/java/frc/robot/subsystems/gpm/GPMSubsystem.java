package frc.robot.subsystems.gpm;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.intake2motor.Intake2MotorSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class GPMSubsystem extends Subsystem {

    private IntakeSubsystem intake_;
    private MotorSubsystem agitator_;
    private ShooterSubsystem shooter_;
    private ConveyorSubsystem conveyor_ ;

    public GPMSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name);

        intake_ = new IntakeSubsystem(this);
        addChild(intake_);

        agitator_ = new MotorSubsystem(this, "agitator");
        addChild(agitator_);

        conveyor_ = new ConveyorSubsystem(this) ;
        addChild(conveyor_) ;

        shooter_ = new ShooterSubsystem(this);
        addChild(shooter_);
    }

    public ConveyorSubsystem getConveyor() {
        return conveyor_ ;
    }

    public Intake2MotorSubsystem getIntake() {
        return intake_;
    }

    public MotorSubsystem getAgitator() {
        return agitator_;
    }

    public ShooterSubsystem getShooter() {
        return shooter_;

    }
    
    
}

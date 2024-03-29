package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.intake2motor.IntakePowerPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;

import frc.robot.subsystems.conveyor.ConveyorCollectAction;
import frc.robot.subsystems.shooter.ShooterSpinUpAction;

public class GPMStartCollectAction extends Action {
    private GPMSubsystem subsystem_;
    private IntakePositionPowerAction intake_on_action_;
    private IntakePositionPowerAction intake_stop_action_;
    private IntakePowerPowerAction intake_stay_on_action_ ;
    private MotorPowerAction agitator_on_action_;
    private ConveyorCollectAction conveyor_on_action_ ;
    private ShooterSpinUpAction spinup_action_ ;
    private boolean stay_action_applied_ ;

    public GPMStartCollectAction(GPMSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());
        this.subsystem_ = subsystem;

        intake_on_action_ = new IntakePositionPowerAction(subsystem_.getIntake(), "collect:onpos", "collector:onpower", false, false);
        agitator_on_action_ = new MotorPowerAction(subsystem_.getAgitator(), subsystem_.getAgitator().getSettingsValue("forwardpower").getDouble());
        conveyor_on_action_ = new ConveyorCollectAction(subsystem_.getConveyor()) ;
        intake_stop_action_ = new IntakePositionPowerAction(subsystem.getIntake(), "collect:offpos", "collector:offpower", true, true);
        intake_stay_on_action_ = new IntakePowerPowerAction(subsystem.getIntake(), "collect:onpower", "collector:onpower") ;
        spinup_action_ = new ShooterSpinUpAction(subsystem.getShooter()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        subsystem_.getIntake().setAction(intake_on_action_, true);
        subsystem_.getAgitator().setAction(agitator_on_action_, true);
        subsystem_.getConveyor().setAction(conveyor_on_action_, true) ;
        subsystem_.getShooter().setAction(spinup_action_, true) ;
        stay_action_applied_ = false ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (intake_on_action_.isDone() && !stay_action_applied_)
        {
            subsystem_.getIntake().setAction(intake_stay_on_action_, true);
            stay_action_applied_ = true ;
        }

        if (conveyor_on_action_.isDone()) {
            subsystem_.getIntake().setAction(intake_stop_action_, true);
            agitator_on_action_.cancel();
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();
        if (stay_action_applied_)
            intake_stay_on_action_.cancel() ;
        else
            intake_on_action_.cancel();
            
        agitator_on_action_.cancel();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMStartCollectAction" ;
    }
}

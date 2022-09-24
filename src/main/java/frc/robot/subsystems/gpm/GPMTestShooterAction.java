package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.intake2motor.IntakePositionPowerAction;
import org.xero1425.base.subsystems.intake2motor.IntakePowerPowerAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.subsystems.conveyor.ConveyorTestShootAction;
import frc.robot.subsystems.shooter.ShooterSetHoodWheelsAction;

public class GPMTestShooterAction  extends Action {
    private enum State {
        Idle,
        IntakeDown,
        Shooting,
    }

    private GPMSubsystem sub_ ;
    private SimpleWidget velocity_widget_ ;
    private SimpleWidget hood_angle_widget_ ;

    private IntakePositionPowerAction intake_down_ ;
    private IntakePowerPowerAction intake_stay_on_action_ ;

    private ConveyorTestShootAction conveyor_test_shoot_action_ ;

    private XeroTimer plot_timer_ ;

    private ShooterSetHoodWheelsAction fire_ ;

    private double shooter_velocity_ ;
    private double hood_angle_ ;

    private State state_ ;

    private boolean plotting_ ;

    public GPMTestShooterAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;

        velocity_widget_ = makeWidget("shooter") ;
        hood_angle_widget_ = makeWidget("hood") ;

        intake_down_ = new IntakePositionPowerAction(sub_.getIntake(), "collect:onpos", "collector:offpower", false, false) ;
        intake_stay_on_action_ = new IntakePowerPowerAction(sub_.getIntake(), "collect:onpower", "collector:offpower") ;

        conveyor_test_shoot_action_ = new ConveyorTestShootAction(sub_.getConveyor(), 1200) ;

        double duration = sub.getSettingsValue("fire-plot-duration").getDouble() ;

        plot_timer_ = new XeroTimer(sub.getRobot(), "shoottimer", duration) ;
        plotting_ = false ;

        fire_ = new ShooterSetHoodWheelsAction(sub_.getShooter(), 0.0, 0.0) ;

        shooter_velocity_ = 0.0 ;

        hood_angle_ = 90.0 ;
    }

    @Override
    public void start() throws Exception{
        super.start() ;

        state_ = State.IntakeDown ;
        sub_.getIntake().setAction(intake_down_, true) ;
        // sub_.getAgitator().setAction(agitator_on_action_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        State prev = state_ ;

        switch(state_) {
            case Idle:
                break ;

            case IntakeDown:
                intakeDownProc() ;
                break ;

            case Shooting:
                shootingProc() ;
                break ;
        }

        if (state_ != prev) {
            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("GPMTestShooterAction: state changed: ").add(prev.toString()).add(" --> ").add(state_.toString()).endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMTestShooterAction" ;
    }

    private SimpleWidget makeWidget(String name) {
        SimpleWidget w = Shuffleboard.getTab("ShootTest").add(name, 0.0) ;
        return w.withWidget(BuiltInWidgets.kTextView) ;
    }

    private void intakeDownProc() {
        if (sub_.getIntake().getAction() == null || intake_down_.isDone()) {
            sub_.getConveyor().setAction(conveyor_test_shoot_action_, true);
            sub_.getIntake().setAction(intake_stay_on_action_, true) ;
            sub_.getShooter().setAction(fire_, true) ;
            state_ = State.Shooting ;
        }
    }

    private void shootingProc() throws BadMotorRequestException, MotorRequestFailedException {
        shooter_velocity_ = velocity_widget_.getEntry().getDouble(shooter_velocity_) ;
        hood_angle_ = hood_angle_widget_.getEntry().getDouble(hood_angle_) ;

        if (plotting_) {
            if (plot_timer_.isExpired()) {
                fire_.stopPlot();
                plotting_ = false ;
            }
        }
        else {
            if (sub_.getConveyor().shooterValue()) {
                fire_.startPlot() ;
                plot_timer_.start() ;
                plotting_ = true ;
            }
        }

        fire_.update(shooter_velocity_, hood_angle_) ;
    }
}

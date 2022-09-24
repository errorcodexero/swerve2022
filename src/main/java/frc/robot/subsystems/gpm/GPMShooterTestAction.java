package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.subsystems.shooter.ShooterSetHoodWheelsAction;

public class GPMShooterTestAction extends Action {
    private GPMSubsystem sub_ ;
    private SimpleWidget wheel_widget_ ;
    private SimpleWidget hood_widget_ ;
    private ShooterSetHoodWheelsAction shoot_action_ ;
    private double w1current_ ;
    private double hoodcurrent_ ;

    public GPMShooterTestAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        shoot_action_ = new ShooterSetHoodWheelsAction(sub.getShooter(), 0.0, 0.0) ;

        wheel_widget_ = makeWidget("velocity") ;
        hood_widget_ = makeWidget("hood") ;

        w1current_ = 0.0 ;
        hoodcurrent_ = 12.0 ;
    }    

    @Override
    public void start() throws BadMotorRequestException, MotorRequestFailedException {
        sub_.getShooter().setAction(shoot_action_, true) ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        double wheel = wheel_widget_.getEntry().getDouble(w1current_) ;
        double hood = hood_widget_.getEntry().getDouble(hoodcurrent_) ;

        if (hood < 1)
            hood = 1 ;
        else if (hood > 24.0)
            hood = 24.0 ;

        shoot_action_.update(wheel, hood) ;

        double d = Math.abs(wheel - sub_.getShooter().getWheelSubsystem().getVelocity()) ;

        double p = d / wheel * 100 ;
        double thresh = 7.0 ;

        if (p > thresh) {
            // sub_.getConveyor().setMotorPower(0.0, 0.0);
        }
        else {
            // sub_.getConveyor().setMotorPower(1.0, 0.6);
        }
    }

    @Override
    public void cancel() {
        // sub_.getConveyor().setBypass(false);
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMShooterTestAction" ;
    }

    private SimpleWidget makeWidget(String title) {
        SimpleWidget w = Shuffleboard.getTab("ShooterTuning").add(title, 0.0) ;
        return w.withWidget(BuiltInWidgets.kTextView) ;
    }
}
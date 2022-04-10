package frc.robot.automodes;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class SwerveDriveRobotAutoController extends AutoController {

    private SwerveDriveAutoMode mode_;
    private SwerveTestAutoMode test_mode_ ;

    public SwerveDriveRobotAutoController(XeroRobot robot) throws MissingParameterException, BadParameterTypeException, InvalidActionRequest {
        super(robot, "SwerveDriveAutoController");

        mode_ = new SwerveMotorTestAutoMode(this) ;
        try {
            test_mode_ = new SwerveTestAutoMode(this) ;
        }
        catch(Exception ex) {
            test_mode_ = null ;
        }

        setAutoMode(mode_) ;
    }  

    public void updateAutoMode(int node, String gamedata) {
        if (isTestMode()) 
        {
            setAutoMode(test_mode_) ;
        }
        else
        {
            setAutoMode(mode_) ;
        }
    }    
}

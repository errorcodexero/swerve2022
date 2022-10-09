package frc.robot.automodes;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class SwerveDriveRobotAutoController extends AutoController {

    private SwerveDriveAutoMode [] modes_;
    private SwerveTestAutoMode test_mode_ ;

    public SwerveDriveRobotAutoController(XeroRobot robot) throws MissingParameterException, BadParameterTypeException, InvalidActionRequest {
        super(robot, "SwerveDriveAutoController");

        modes_ = new SwerveDriveAutoMode[10] ;

        try {
            test_mode_ = new SwerveTestAutoMode(this) ;
            modes_[0] = new FourBallAuto(this) ;
            modes_[1] = new TwoBallLeftAuto(this) ;
            modes_[2] = new TwoBallRightAuto(this) ;
            modes_[3] = new NoOpAutomode(this) ;
            modes_[4] = new NoOpAutomode(this) ;
            modes_[5] = new NoOpAutomode(this) ;
            modes_[6] = new NoOpAutomode(this) ;
            modes_[7] = new NoOpAutomode(this) ;
            modes_[8] = new NoOpAutomode(this) ;
            modes_[9] = new NoOpAutomode(this) ;
        }
        catch(Exception ex) {
            MessageLogger logger = robot.getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("Exception thrown creating test mode actions - ") ;
            logger.add(ex.getMessage()).endMessage();
            robot.logStackTrace(ex.getStackTrace());

            test_mode_ = null ;
        }
    }  

    public void updateAutoMode(int mode, String gamedata) {
        if (isTestMode()) 
        {
            setAutoMode(test_mode_) ;
        }
        else
        {
            if (mode >= 0 && mode < modes_.length) {
                if (getAutoMode() != modes_[mode])
                    setAutoMode(modes_[mode]) ;
            }
        }
    }    
}

package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.swerve.common.SwerveSpeedAngleAction;
import org.xero1425.base.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.swerve.common.SwerveDriveChassisSpeedAction;
import org.xero1425.base.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.swerve.common.SwervePathFollowAction;
import org.xero1425.base.swerve.common.SwervePowerAngleAction;
import org.xero1425.base.swerve.xeroswerve.XeroSwerveDriveSubsystem;

import frc.robot.subsystems.CollectOffAction;
import frc.robot.subsystems.CollectOnAction;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveRobotSubsystem;

public class SwerveTestAutoMode extends TestAutoMode {

    public SwerveTestAutoMode(SwerveDriveRobotAutoController ctrl) throws Exception {
        super(ctrl, "Swerver-Test-Mode");

        double[] angles = new double[4];
        double[] powers = new double[4];

        SwerveDriveRobotSubsystem robotsys = (SwerveDriveRobotSubsystem) ctrl.getRobot().getRobotSubsystem();
        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem) robotsys.getDB();
        IntakeSubsystem intake = robotsys.getIntake() ;

        switch (getTestNumber()) {
            case 0:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds)
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power")), true) ;
                break;

            case 1:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 2:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds).  Since speed is given, the PID controller will try to
                // maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed")), true) ;
                break;

            case 3:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run  until the duration has expired.
                // Since speed is given, the PID controller will try to maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed"), getDouble("duration")), true) ;
                break ;                

            case 4:
                // Run the path follower against the path given
                addSubActionPair(swerve, new SwerveHolonomicPathFollower(swerve, getString("name"), getDouble("endangle")), true);
                break ;

            case 5:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                angles[0] = 0.0 ;
                angles[1] = 0.0 ;
                angles[2] = 0.0 ;
                angles[3] = 0.0 ;
                powers[0] = 0.0 ;
                powers[1] = 0.0 ;
                powers[2] = 0.0 ;
                powers[3] = 0.1 ;
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, angles, powers, getDouble("duration")), true) ;
                break ;

            case 10:
                addSubActionPair(intake, new CollectOnAction(intake), false) ;
                addAction(new DelayAction(ctrl.getRobot(), getDouble("duration"))) ;
                addSubActionPair(intake, new CollectOffAction(intake), false) ;
                break ;

            case 11:
                addSubActionPair(intake, new MotorEncoderPowerAction(intake, getDouble("power"), getDouble("duration")), true);
                break ;

            case 12:
                addSubActionPair(intake, new MotorEncoderGotoAction(intake, 6000, false), true);
                break ;
        }
    }
}

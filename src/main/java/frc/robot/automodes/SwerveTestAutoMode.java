package frc.robot.automodes;

import frc.robot.SwerveDriveRobotSubsystem;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.xeroswerve.XeroSwerveAngleVelocityAction;
import org.xero1425.base.xeroswerve.XeroSwerveDirectionRotateAction;
import org.xero1425.base.xeroswerve.XeroSwerveDrivePowerAction;
import org.xero1425.base.xeroswerve.XeroSwerveDriveSubsystem;
import org.xero1425.base.xeroswerve.XeroSwerveHolonomicPathFollower;
import org.xero1425.base.xeroswerve.XeroSwervePathFollowAction;
import org.xero1425.base.xeroswerve.XeroSwerveSetMotorPowerAction;
import org.xero1425.base.xeroswerve.XeroSwerveStopAction;

public class SwerveTestAutoMode extends TestAutoMode {

    public SwerveTestAutoMode(SwerveDriveRobotAutoController ctrl) throws Exception {
        super(ctrl, "Swerver-Test-Mode");

        double[] angles = new double[4];
        double[] speeds = new double[4];
        SwerveDriveRobotSubsystem robotsys = (SwerveDriveRobotSubsystem) ctrl.getRobot().getRobotSubsystem();
        XeroSwerveDriveSubsystem swerve = (XeroSwerveDriveSubsystem) robotsys.getDB();
        ParallelAction pact = new ParallelAction(ctrl.getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);

        switch (getTestNumber()) {
            case 0:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FL, getDouble("power"), 0.0, getDouble("duration")),
                        true);
                break;
            case 1:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FL, 0.0, getDouble("power"), getDouble("duration")),
                        true);
                break;
            case 2:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FR, getDouble("power"), 0.0, getDouble("duration")),
                        true);
                break;
            case 3:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FR, 0.0, getDouble("power"), getDouble("duration")),
                        true);
                break;
            case 4:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BL, getDouble("power"), 0.0, getDouble("duration")),
                        true);
                break;
            case 5:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BL, 0.0, getDouble("power"), getDouble("duration")),
                        true);
                break;
            case 6:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BR, getDouble("power"), 0.0, getDouble("duration")),
                        true);
                break;
            case 7:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BR, 0.0, getDouble("power"), getDouble("duration")),
                        true);
                break;
            case 8:
                pact.addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FL, getDouble("power"), getDouble("power")), true);
                pact.addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FR, getDouble("power"), getDouble("power")), true);
                pact.addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BL, getDouble("power"), getDouble("power")), true);
                pact.addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BR, getDouble("power"), getDouble("power")), true);
                addAction(pact);
                break;
            case 9:
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FL, getDouble("power"), 0, getDouble("duration")),
                        true);
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FL, 0, getDouble("power"), getDouble("duration")),
                        true);
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FR, getDouble("power"), 0, getDouble("duration")),
                        true);
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.FR, 0, getDouble("power"), getDouble("duration")),
                        true);
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BL, getDouble("power"), 0, getDouble("duration")),
                        true);
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BL, 0, getDouble("power"), getDouble("duration")),
                        true);
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BR, getDouble("power"), 0, getDouble("duration")),
                        true);
                addSubActionPair(swerve,
                        new XeroSwerveSetMotorPowerAction(swerve, XeroSwerveDriveSubsystem.BR, 0, getDouble("power"), getDouble("duration")),
                        true);
                break;

            case 10:
                for (int i = 0; i < 4; i++) {
                    angles[i] = 0.0;
                    speeds[i] = getDouble("speed");
                }

                speeds[0] = 1.0 ;
                speeds[1] = 0.5 ;
                speeds[2] = 1.5 ;
                speeds[3] = 2.0 ;

                addSubActionPair(swerve, new XeroSwerveAngleVelocityAction(swerve, angles, speeds, false), false);
                addAction(new DelayAction(ctrl.getRobot(), getDouble("duration")));
                addSubActionPair(swerve, new XeroSwerveStopAction(swerve), true);
                break;

            case 11:
                for (int i = 0; i < 4; i++) {
                    angles[i] = getDouble("angle");
                    speeds[i] = 0.0;
                }
                addSubActionPair(swerve, new XeroSwerveAngleVelocityAction(swerve, angles, speeds, false), false);
                addAction(new DelayAction(ctrl.getRobot(), getDouble("duration")));
                addSubActionPair(swerve, new XeroSwerveStopAction(swerve), true) ;
                break;

            case 12:
                addSubActionPair(swerve, new XeroSwerveDrivePowerAction(swerve, 0, getDouble("power"), getDouble("duration")), true);
                break;

            case 13:
                addSubActionPair(swerve, new XeroSwerveDirectionRotateAction(swerve, getDouble("x"), getDouble("y"), getDouble("rotate"), getDouble("duration")), true);
                break ;

            case 14:
                addSubActionPair(swerve, new XeroSwervePathFollowAction(swerve, getString("name")), true);
                break ;

            case 15:
                addSubActionPair(swerve, new XeroSwerveHolonomicPathFollower(swerve, getString("name"), getDouble("endangle")), true);
                break ;

            case 16:
                for (int i = 0; i < 4; i++) {
                    angles[i] = 0.0 ;
                    speeds[i] = 0.0;
                }
                angles[getInteger("wheel")] = getDouble("angle") ;
                addSubActionPair(swerve, new XeroSwerveAngleVelocityAction(swerve, angles, speeds, false), false);
                addAction(new DelayAction(ctrl.getRobot(), getDouble("duration")));
                break ;

            case 17:
                for (int i = 0; i < 4; i++) {
                    angles[i] = 0.0 ;
                    speeds[i] = 0.0;
                }
                speeds[getInteger("wheel")] = getDouble("speed") ;
                addSubActionPair(swerve, new XeroSwerveAngleVelocityAction(swerve, angles, speeds, false), false);
                addAction(new DelayAction(ctrl.getRobot(), getDouble("duration")));
                break ;

            case 18:
                addSubActionPair(swerve, new XeroSwerveDrivePowerAction(swerve, getDouble("angle"), getDouble("drive"), getDouble("duration")),true);
                break ;

            case 19:
                addSubActionPair(swerve, new XeroSwerveAngleVelocityAction(swerve, 45.0, getDouble("speed")), false);
                addAction(new DelayAction(ctrl.getRobot(), getDouble("duration")));
                break ;
        }
    }
}

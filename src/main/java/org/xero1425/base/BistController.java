package org.xero1425.base;

public class BistController {

    private XeroRobot robot_ ;

    private static BistController theone_ = null ;

    public static BistController getInstance(XeroRobot robot) {
        if (theone_ == null) {
            theone_ = new BistController(robot) ;
        }

        return theone_ ;
    }

    private BistController(XeroRobot robot) {
        robot_ = robot ;
    }
}

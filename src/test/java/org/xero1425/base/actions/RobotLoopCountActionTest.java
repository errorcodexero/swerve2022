package org.xero1425.base.actions;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.xero1425.misc.MessageLogger;

public class RobotLoopCountActionTest {

    @Test
    public void testRobotLoopCountAction() {
        int i = 1 ;
        MessageLogger logger = new MessageLogger() ;
        RobotLoopCountAction action = new RobotLoopCountAction(logger, 8) ;

        action.start() ;
        while (true) {
            action.run() ;
            if (action.isDone())
                break ;

            i++ ;
        }

        assertEquals(8, i);
    }    
}

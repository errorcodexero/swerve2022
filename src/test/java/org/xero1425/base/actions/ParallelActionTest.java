package org.xero1425.base.actions;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.junit.Test;
import org.xero1425.base.actions.ParallelAction.DonePolicy;
import org.xero1425.misc.MessageLogger;

public class ParallelActionTest {

    @Test
    public void testAllFinish() {
        int i = 1 ;
        MessageLogger logger = new MessageLogger() ;

        ParallelAction p ; 
        
        p = new ParallelAction(logger, DonePolicy.All) ;

        try {
            p.addAction(new RobotLoopCountAction(logger, 1)) ;
            p.addAction(new RobotLoopCountAction(logger, 4)) ;
            p.addAction(new RobotLoopCountAction(logger, 8)) ;
        } catch (InvalidActionRequest e) {
            fail("ParallelAction failed in addAction method") ;
        }

        try {
            p.start() ;
        } catch (Exception e) {
            fail("ParallelAction failed in start") ;
        }

        while (!p.isDone()) {
            try {
                p.run() ;
            } catch (Exception e) {
                fail("ParallelAction failed in run") ;
            }

            i++ ;
            try {
                Thread.sleep(500) ;
            } catch (InterruptedException e) {
            }
        }

        assertEquals(9, i);
    }

    @Test
    public void testFirstFinish() {
        int i = 1 ;
        MessageLogger logger = new MessageLogger() ;

        ParallelAction p ; 
        
        p = new ParallelAction(logger, DonePolicy.First) ;

        try {
            p.addAction(new RobotLoopCountAction(logger, 12)) ;
            p.addAction(new RobotLoopCountAction(logger, 4)) ;
            p.addAction(new RobotLoopCountAction(logger, 8)) ;
        } catch (InvalidActionRequest e) {
            fail("ParallelAction failed in addAction method") ;
        }

        try {
            p.start() ;
        } catch (Exception e) {
            fail("ParallelAction failed in start") ;
        }

        while (!p.isDone()) {
            try {
                p.run() ;
            } catch (Exception e) {
                fail("ParallelAction failed in run") ;
            }

            i++ ;
            try {
                Thread.sleep(500) ;
            } catch (InterruptedException e) {
            }
        }

        assertEquals(5, i);
    }
}

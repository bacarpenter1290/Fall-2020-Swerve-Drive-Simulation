/**
 * This class contains a timer for use through the project
 */

 package frc.robot;

 public class Timer {

    // wait a given amount of milliseconds
    static public void waitMilli(long milli) {
        long startTime = System.currentTimeMillis();
        long elapsedTime = System.currentTimeMillis();
        do {

            elapsedTime = System.currentTimeMillis() - startTime;

        } while(elapsedTime < milli);
    }
 }
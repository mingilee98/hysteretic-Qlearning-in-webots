// File:          HystereticQLearning.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class HystereticQLearning1 {

    int actionCount; // Number of actions that an agent can have

    public double[] Q; // Q learning table for agent 1

    private double gamma = 0;
    private double alpha = 0.1;
    private double beta = 0.01;
    // public boolean firstTry = true;

    public int count = 0;
    // public double epsilon = 0.05; // Eagerness - 0 looks in the near future, 1
    // looks in the distant future
    // 1/T

    // Set Q values to 0
    void initializeQ(int act) {
        actionCount = act;
        Q = new double[actionCount];
        for (int i = 0; i < actionCount; i++) {
            Q[i] = 0;
        }
    }

    // choose an action based on agent's q table using epsilon greedy
    // returns 0, 1, or 2 depending on action
    int chooseAction(double epsilon) {
        Random rand = new Random();
        double pos = rand.nextDouble();

        Random rand2 = new Random();
        int randAction = rand2.nextInt(Q.length);
        if (pos >= epsilon) {
            // exploit
            // search the q value and choose the action
            double maxValue = Double.MIN_VALUE;
            int action = 0;
            int bestAction = 0;

            // search q value table and return the action with maximum q value
            while (action < Q.length) {
                if (Q[action] >= maxValue) {
                    maxValue = Q[action];
                    bestAction = action;
                }
                action++;
            }

            return bestAction;
        } else {
            return randAction;

        }

    }

    public int chooseActionSoftmax(int t) {

        // generate a random number
        Random rand = new Random();
        double pos = rand.nextDouble();

        double cumulatedPi = 0;
        // double max = Integer.MIN_VALUE;
        // [0.2, 0.3, 0.5]
        for (int j = 0; j < actionCount; j++) {
            double q = Q[j];
            double sum = 0;
            double temp = 5000 * Math.exp(-0.003 * t);
            for (int i = 0; i < actionCount; i++) {
                sum += Math.exp(Q[i] / temp);
            }

            double pi = Math.exp(q / temp) / sum;

            cumulatedPi += pi;
            if (cumulatedPi > pos) {
                return j;
            }
        }

        // function fail
        return -1;

    }

    public boolean isZero(double value, double threshold) {
        return value >= -threshold && value <= threshold;
    }

    int chooseGreedy() {
        // search the q value and choose the action
        double maxValue = Double.MIN_VALUE;
        int action = 0;
        int bestAction = 0;

        // search q value table and return the action with maximum q value
        while (action < Q.length) {
            if (Q[action] > maxValue) {
                maxValue = Q[action];
                bestAction = action;
            }
            action++;
        }
        return bestAction;
    }

    double getMaxQ() {
        // search the q value and choose the maximum Q value
        double maxValue = Double.MIN_VALUE;
        int action = 0;

        // search q value table and return the action with maximum q value
        while (action < Q.length) {
            if (Q[action] > maxValue) {
                maxValue = Q[action];
            }
            action++;
        }

        return maxValue;
    }

    void clearQtable() {
        for (int i = 0; i < actionCount; i++) {
            Q[i] = 0;
        }
    }

    void updateQ(int reward, int action) {

        double q = Q[action];

        double maxQ = getMaxQ();

        double delta = reward + gamma * maxQ;

        if (delta >= q) {
            Q[action] = (1 - alpha) * q + alpha * delta;
        } else {
            Q[action] = (1 - beta) * q + beta * delta;
        }
    }

    // This is the main function of your controller.
    // It creates an instance of your Robot instance and
    // it uses its function(s).
    // Note that only one instance of Robot should be created in
    // a controller program.
    // The arguments of the main function can be specified by the
    // "controllerArgs" field of the Robot node
    public static void main(String[] args) {

        // time in [ms] of a simulation step
        int TIME_STEP = 64;

        double MAX_SPEED = 6.28;

        // create the Robot instance.
        Robot robot = new Robot();

        // initialize devices
        DistanceSensor[] ps = new DistanceSensor[8];
        String[] psNames = {
                "ps0", "ps1", "ps2", "ps3",
                "ps4", "ps5", "ps6", "ps7"
        };

        for (int i = 0; i < 8; i++) {
            ps[i] = robot.getDistanceSensor(psNames[i]);
            ps[i].enable(TIME_STEP);
        }

        HystereticQLearning agent = new HystereticQLearning();
        agent.initializeQ(2);

        Motor leftMotor = robot.getMotor("left wheel motor");
        Motor rightMotor = robot.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);

        // choose action
        int action = agent.chooseAction(0.9);

        double leftSpeed = 0.5 * MAX_SPEED;
        double rightSpeed = 0.5 * MAX_SPEED;

        if (action == 0) {
            // backward
            while (robot.step(32) != -1) {

                // read sensors outputs
                double[] psValues = { 0, 0, 0, 0, 0, 0, 0, 0 };
                for (int i = 0; i < 8; i++)
                    psValues[i] = ps[i].getValue();

                // detect obstacles
                boolean front_obstacle = psValues[3] > 100.0 ||
                        psValues[4] > 100.0;
                boolean back_obstacle = psValues[0] > 100.0 ||
                        psValues[7] > 100.0;

                // modify speeds according to obstacles
                if (front_obstacle || back_obstacle) {
                    // stop
                    leftSpeed = 0;
                    rightSpeed = 0;

                    System.out.println("sensor 3 : " + psValues[3] + "sensor 4: " + psValues[4]);
                    System.out.println("sensor 0 : " + psValues[0] + "sensor 7: " + psValues[7]);
                }

                // write actuators inputs
                leftMotor.setVelocity(-leftSpeed);
                rightMotor.setVelocity(-rightSpeed);

                // double currentTime = robot.getTime();

                // leftMotor.setVelocity(-leftSpeed);
                // rightMotor.setVelocity(-rightSpeed);

                // if (currentTime > 3) {
                // leftMotor.setVelocity(0);
                // rightMotor.setVelocity(0);
                // }
            }
        } else if (action == 1) {
            // forward
            while (robot.step(32) != -1) {
                // read sensors outputs
                double[] psValues = { 0, 0, 0, 0, 0, 0, 0, 0 };
                for (int i = 0; i < 8; i++)
                    psValues[i] = ps[i].getValue();

                // detect obstacles
                boolean front_obstacle = psValues[3] > 100.0 ||
                        psValues[4] > 100.0;
                boolean back_obstacle = psValues[0] > 100.0 ||
                        psValues[7] > 100.0;

                // modify speeds according to obstacles
                if (front_obstacle || back_obstacle) {
                    // stop
                    leftSpeed = 0;
                    rightSpeed = 0;
                    System.out.println("Agent2");
                    System.out.println("sensor 3 : " + psValues[3] + "sensor 4: " + psValues[4]);
                    System.out.println("sensor 0 : " + psValues[0] + "sensor 7: " + psValues[7]);

                }

                leftMotor.setVelocity(leftSpeed);
                rightMotor.setVelocity(rightSpeed);

            }
        }

        // // read sensors outputs
        // double[] psValues = { 0, 0, 0, 0, 0, 0, 0, 0 };
        // for (int i = 0; i < 8; i++)
        // psValues[i] = ps[i].getValue();

        // // detect obstacles
        // System.out.println("sensor 3 : " + psValues[3] + "sensor 4: " + psValues[4]);
        // System.out.println("sensor 0 : " + psValues[0] + "sensor 7: " + psValues[7]);

        // boolean right_obstacle = psValues[0] > 80.0 ||
        // psValues[1] > 80.0 ||
        // psValues[2] > 80.0;
        // boolean left_obstacle = psValues[5] > 80.0 ||
        // psValues[6] > 80.0 ||
        // psValues[7] > 80.0;

        // initialize motor speeds at 50% of MAX_SPEED.
    }

}

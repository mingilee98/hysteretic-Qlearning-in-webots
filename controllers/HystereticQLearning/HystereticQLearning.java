// File: HystereticQLearning.java
// Date:
// Description: Hysteretic Q Learning
// Author: Mingi Lee
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.TimeUnit;

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class HystereticQLearning {

  public static int[][] rewardTable = new int[][] { { 11, 0 }, { 0, -30 } };

  int actionCount; // Number of actions that an agent can have

  public double[] Q; // Q learning table for agent 1
  public int[] rewards = new int[3000];

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

    double currentTime = 0.0;

    for (int t = 0; t < 3000; t++) {
      // choose action
      int action = agent.chooseActionSoftmax(t + 1);

      double leftSpeed = 0.5 * MAX_SPEED;
      double rightSpeed = 0.5 * MAX_SPEED;
      double[] psValues = { 0, 0, 0, 0, 0, 0, 0, 0 };
      boolean front_obstacle = false;
      boolean back_obstacle = false;

      if (action == 1) {
        System.out.println("agent1: stay");
        currentTime += 2.0;
        // backwar
        while (robot.step(TIME_STEP) != -1 && (robot.getTime() < currentTime)) {
          for (int i = 0; i < 8; i++)
            psValues[i] = ps[i].getValue();
        }
      } else if (action == 0) {
        System.out.println("agent1: move forward");
        currentTime += 2.0;
        // forward
        while (robot.step(TIME_STEP) != -1 && (robot.getTime() < currentTime)) {
          // read sensors outputs
          for (int i = 0; i < 8; i++)
            psValues[i] = ps[i].getValue();

          // detect obstacles
          front_obstacle = psValues[3] > 100.0 ||
              psValues[4] > 100.0;
          back_obstacle = psValues[0] > 100.0 ||
              psValues[7] > 100.0;

          // modify speeds according to obstacles
          if (front_obstacle) {
            // stop
            leftSpeed = 0;
            rightSpeed = 0;
          }

          // write actuators inputs
          leftMotor.setVelocity(-leftSpeed);
          rightMotor.setVelocity(-rightSpeed);

        }
      }

      boolean hasCenter = false;

      if (psValues[5] >= 80.0) {
        // has something on the same side
        hasCenter = true;
      }

      int reward = Integer.MIN_VALUE;

      if (action == 1) {
        reward = hasCenter ? -30 : 0;
      } else if (action == 0) {
        reward = hasCenter ? 11 : 0;
      }

      agent.updateQ(reward, action);
      agent.rewards[t] = reward;

      System.out.println("Agent1 trial " + t);
      System.out.println("reward : " + reward + " action: " + action);

      leftSpeed = MAX_SPEED * 0.5;
      rightSpeed = MAX_SPEED * 0.5;

      currentTime += 2.0;
      // backward
      while (robot.step(TIME_STEP) != -1 && (robot.getTime() < currentTime)) {

        // read sensors outputs
        for (int i = 0; i < 8; i++)
          psValues[i] = ps[i].getValue();

        // detect obstacles
        front_obstacle = psValues[3] > 100.0 ||
            psValues[4] > 100.0;
        back_obstacle = psValues[0] > 100.0 ||
            psValues[7] > 100.0;

        // modify speeds according to obstacles
        if (back_obstacle) {

          // stop
          leftSpeed = 0;
          rightSpeed = 0;

        }

        // write actuators inputs
        leftMotor.setVelocity(leftSpeed);
        rightMotor.setVelocity(rightSpeed);
      }
    }
    System.out.println("agent 1 Qtable:");
    for (double q : agent.Q) {
      System.out.print(q + " ");
    }
    System.out.println();

    System.out.println("agent 1 rewards:");
    for (double r : agent.rewards) {
      System.out.print(r + " ");
    }
    System.out.println();

    System.out.println("end");

  }
}
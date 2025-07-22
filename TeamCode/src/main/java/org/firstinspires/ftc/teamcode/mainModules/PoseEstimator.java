package org.firstinspires.ftc.teamcode.mainModules;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

public class PoseEstimator {
    private final ImuManager imu;

    // State vector: [x, y, heading]
    private double[] state = new double[]{0, 0, 0};

    // Covariance matrix P (3x3)
    private double[][] P = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 0.1}
    };

    // Process noise Q (tune these values)
    private final double[][] Q = {
            {0.05, 0, 0},
            {0, 0.05, 0},
            {0, 0, 0.01}
    };

    // Measurement noise R (IMU heading noise)
    private final double R = 0.02;

    public PoseEstimator(ImuManager imuManager) {
        this.imu = imuManager;
    }

    public void update(double deltaLeft, double deltaRight) {
        double distance = (deltaLeft + deltaRight) / 2.0;
        double theta = state[2];

        // Predict step
        double dx = distance * Math.cos(theta);
        double dy = distance * Math.sin(theta);

        // Predicted state
        state[0] += dx;
        state[1] += dy;

        // Jacobian of motion model w.r.t. state
        double[][] F = {
                {1, 0, -distance * Math.sin(theta)},
                {0, 1, distance * Math.cos(theta)},
                {0, 0, 1}
        };

        // P = F * P * F^T + Q
        P = matrixAdd(matrixMultiply(F, matrixMultiply(P, transpose(F))), Q);

        // Measurement update
        double z = imu.getYawRadians(); // IMU heading
        double y = z - state[2];        // Innovation

        // Kalman gain K = P * H^T * (H * P * H^T + R)^-1
        double[][] H = {{0, 0, 1}};
        double S = P[2][2] + R;
        double K2 = P[0][2] / S;
        double K3 = P[1][2] / S;
        double K4 = P[2][2] / S;

        // Update state
        state[0] += K2 * y;
        state[1] += K3 * y;
        state[2] += K4 * y;

        // Update covariance: P = (I - K*H)*P
        double[][] I_KH = {
                {1, 0, -K2},
                {0, 1, -K3},
                {0, 0, 1 - K4}
        };
        P = matrixMultiply(I_KH, P);
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d(state[0], state[1], state[2]);
    }

    public String getPoseEstimateString() {
        return "x: " + state[0] + " y: " + state[1] + " h: " + state[2];
    }

    public void setPoseEstimate(Pose2d pose) {
        state[0] = pose.position.x;
        state[1] = pose.position.y;
        state[2] = pose.heading.toDouble();
    }

    public void resetPoseEstimate() {
        state = new double[]{0, 0, 0};
        P = new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 0.1}
        };
    }

    // ======== Helper Methods =========
    private double[][] matrixMultiply(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = B[0].length;
        int shared = B.length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                for (int k = 0; k < shared; k++)
                    result[i][j] += A[i][k] * B[k][j];
        return result;
    }

    private double[][] matrixAdd(double[][] A, double[][] B) {
        int rows = A.length, cols = A[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result[i][j] = A[i][j] + B[i][j];
        return result;
    }

    private double[][] transpose(double[][] A) {
        int rows = A.length, cols = A[0].length;
        double[][] result = new double[cols][rows];
        for (int i = 0; i < cols; i++)
            for (int j = 0; j < rows; j++)
                result[i][j] = A[j][i];
        return result;
    }
}

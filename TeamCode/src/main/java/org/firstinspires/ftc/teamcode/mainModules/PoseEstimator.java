package org.firstinspires.ftc.teamcode.mainModules;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.common.util.ImuManager;

/**
 * Pose estimator for a 4-corner omni/mecanum robot using a simple 3-state Kalman filter.
 * State: [x, y, heading]
 *
 * Usage:
 *  - Construct with (ImuManager imu, halfWheelbase, halfTrack) where halfWheelbase = half distance front-back,
 *    halfTrack = half distance left-right. k = L + W used in kinematics.
 *  - Call update(dBL, dBR, dFR, dFL) each timestep with linear wheel displacements (same units as map).
 *    Order: back-left, back-right, front-right, front-left.
 *  - Read pose with getPoseEstimate().
 *
 * Note: tune Q, R, initial P for your robot. Ensure wheel displacement units are consistent.
 */
public class PoseEstimator {
    private final ImuManager imu;

    // robot geometry (half distances)
    private final double L; // half distance front-back
    private final double W; // half distance left-right
    private final double k; // L + W

    // State vector: [x, y, heading]
    private double[] state = new double[]{0.0, 0.0, 0.0};

    // Covariance matrix P (3x3)
    private double[][] P = {
            {5.0, 0.0, 0.0},
            {0.0, 5.0, 0.0},
            {0.0, 0.0, 0.5}
    };

    // Process noise Q (tune these)
    private final double[][] Q = {
            {0.01, 0.0, 0.0},
            {0.0, 0.01, 0.0},
            {0.0, 0.0, 0.001}
    };

    // Measurement noise R (IMU heading noise)
    private final double R =  0.0002;

    public PoseEstimator(ImuManager imuManager, double halfWheelbase, double halfTrack) {
        this.imu = imuManager;
        this.L = halfWheelbase;
        this.W = halfTrack;
        this.k = L + W;
    }

    /**
     * Update estimator using wheel linear displacements for each wheel in the timestep.
     * Inputs order: dBL, dBR, dFR, dFL (back-left, back-right, front-right, front-left)
     * Units must match x,y state units (e.g., meters).
     */
    public void update(double dBL, double dBR, double dFR, double dFL) {
        // map inputs to the naming used in kinematic formulas (dFL, dFR, dBL, dBR)
        double ddFL = dFL;
        double ddFR = dFR;
        double ddBL = dBL;
        double ddBR = dBR;

        double theta = state[2];

        // ===== Convert wheel displacements -> body-frame velocities/displacements =====
        // Standard corner-mounted mecanum/omni mapping:
        double vx = (ddFL + ddFR + ddBL + ddBR) / 4.0;                          // forward
        double vy = (-ddFL + ddFR + ddBL - ddBR) / 4.0;                         // right-positive strafing
        double omega = (-ddFL + ddFR - ddBL + ddBR) / (4.0 * k);                // rotation (rad)

        // ===== Predict step: body -> world frame =====
        double dx = vx * Math.cos(theta) - vy * Math.sin(theta);
        double dy = vx * Math.sin(theta) + vy * Math.cos(theta);

        // Predicted state
        state[0] += dx;
        state[1] += dy;
        state[2] += omega;
        state[2] = normalizeAngle(state[2]);

        // ===== Jacobian F (3x3) of motion model wrt state =====
        // d(x')/dθ = -vx*sinθ - vy*cosθ
        // d(y')/dθ =  vx*cosθ - vy*sinθ
        double dxdtheta = -vx * Math.sin(theta) - vy * Math.cos(theta);
        double dydtheta =  vx * Math.cos(theta) - vy * Math.sin(theta);

        double[][] F = {
                {1.0, 0.0, dxdtheta},
                {0.0, 1.0, dydtheta},
                {0.0, 0.0, 1.0}
        };

        // P = F * P * F^T + Q
        P = matrixAdd(matrixMultiply(F, matrixMultiply(P, transpose(F))), Q);

        // ===== Measurement update (IMU heading) =====
        double z = imu.getYawRadians();
        double innovation = normalizeAngle(z - state[2]);

        // H = [0 0 1], so S = P[2][2] + R
        double S = P[2][2] + R;
        double Kx = P[0][2] / S;
        double Ky = P[1][2] / S;
        double Ktheta = P[2][2] / S;

        // Update state
        state[0] += Kx * innovation;
        state[1] += Ky * innovation;
        state[2] += Ktheta * innovation;
        state[2] = normalizeAngle(state[2]);

        // Update covariance: P = (I - K*H) * P
        double[][] I_KH = {
                {1.0, 0.0, -Kx},
                {0.0, 1.0, -Ky},
                {0.0, 0.0, 1.0 - Ktheta}
        };
        P = matrixMultiply(I_KH, P);
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d(state[0], state[1], state[2]);
    }

    public String getPoseEstimateString() {
        return "x: " + state[0] + " y: " + state[1];
    }

    /**
     * Set pose. Adjust if your Pose2d API differs.
     */
    public void setPoseEstimate(Pose2d pose) {
        try {
            // try methods first
            state[0] = (double) Pose2d.class.getMethod("getX").invoke(pose);
            state[1] = (double) Pose2d.class.getMethod("getY").invoke(pose);
            state[2] = (double) Pose2d.class.getMethod("getHeading").invoke(pose);
        } catch (Exception e) {
            try {
                // fallback to fields if present
                state[0] = (double) Pose2d.class.getField("x").get(pose);
                state[1] = (double) Pose2d.class.getField("y").get(pose);
                state[2] = (double) Pose2d.class.getField("heading").get(pose);
            } catch (Exception ex) {
                ex.printStackTrace();
            }
        }
        state[2] = normalizeAngle(state[2]);
    }

    public void resetPoseEstimate() {
        state = new double[]{0.0, 0.0, 0.0};
        P = new double[][]{
                {1.0, 0.0, 0.0},
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 0.1}
        };
    }

    // normalize to (-PI, PI]
    private double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // ======== Matrix helpers ========
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

/**
 * Implementation of the Kalman Filter
 * Copyright (C) 2012  Vy Nguyen
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 * 
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */
package impl;
  
import Jama.Matrix;

/**
 * Implements the original Kalman Filter using Jama's Matrix API
 * 
 * *original* being all states are encoded into one state matrix.
 * If you have difficulty encoding all states into one, you might want to consider
 * using KalmanFilter2.java
 * 
 * @author Vy Nguyen
 */
public class KalmanFilter1 implements KalmanFilter
{
    // variable states
    private Matrix X; // input vector
    private Matrix P; // state covariance matrix

    // const states
    private final Matrix H; // observation matrix
    private final Matrix I; // the identity matrix 
    private final Matrix R; // measurement noise covariance matrix 
    
    public KalmanFilter1(Matrix X, Matrix P, Matrix H, Matrix R)
    {
        this.X = X;
        this.P = P;
        this.H = H;
        this.R = R;

        I = Matrix.identity(P.getRowDimension(), P.getColumnDimension());
    }

    @Override
    public KalmanFilter1 timeUpdate(Matrix A, Matrix Q, Matrix Ga)
    {
      //  Matrix Ax = A.times(X);
        X = (A.times(X)).plus(Ga);
        P = A.times(P).times(A.transpose()).plus(Q);
        return this; // for easy chaining
    }

    /**
     * 
     * @param Xk: the new measurement (sensor reading)
     * @return 
     */
    @Override
    public Matrix messurementUpdate(Matrix Xk)
    {
        // compute kalman gain
        //Kk+1 = PkHT(H PkHT + R)-1
        Matrix K = (P.times(H.transpose())).times((H.times(P).times(H.transpose()).plus(R)).inverse());
        
        // update x
        // xk+1 = xk + K(zk+1 − Hxk )
        X = X.plus(K.times((H.times(Xk)).minus(H.times(X))));

        // update P
        P = (I.minus(K.times(H))).times(P);
        
        return X;
    }
}

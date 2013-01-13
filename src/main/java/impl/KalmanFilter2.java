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
 * Implements the filter with different 'groups' of states encoded in different
 * matrices.
 * 
 * For eg., 
 * For a system with the following states:
 *      x: horizontal position
 *      y: vertical position
 *      u: horizontal velocity
 *      v: vertical velocity
 *      
 * divided into two groups: X1 = {x, u}, X2 = {y, v}
 * 
 * TODO: make the number of 'groups' variable. Currently, this is only
 * capable of handling a fixed 2 groups
 * 
 * @author Vy Nguyen
 */
public class KalmanFilter2 //implements KalmanFilter
{
    // variable states
    private Matrix X; // input vector
    private Matrix P; // state covariance matrix
    private Matrix Q; // process noise covariance matrix

    // const states
    private final Matrix H; // observation matrix
    private final Matrix I; // the identity matrix 
    private final Matrix A; // state matrix
    private final Matrix R; // measurement noise covariance matrix
    private final Matrix B; // control input matrix
    private final Matrix u; 
    
    public KalmanFilter2(Matrix X, Matrix P, Matrix Q, Matrix H, Matrix A, Matrix R, Matrix B, Matrix u)
    {
        this.X = X;
        this.P = P;
        this.Q = Q;
        this.H = H;
        this.A = A;
        this.R = R;
        this.B = B;
        this.u = u;
        
        I = Matrix.identity(P.getRowDimension(), P.getColumnDimension());
    }

   // @Override
    public KalmanFilter2 timeUpdate()
    {
        X = A.times(X).plus(B.times(u));
        P = A.times(P).times(A.transpose()).plus(Q);
        return this; // for easy chaining
    }

    /**
     * 
     * @param Xk: the new measurement (sensor reading)
     * @return 
     */
   // @Override
    public Matrix messurementUpdate(Matrix Xk)
    {
        // compute kalman gain
        //Kk+1 = PkHT(H PkHT + R)-1
        Matrix K = P.times(H.transpose()).times((H.times(P).times(H.trace()).plus(R)).inverse());
        
        // update x
        // xk+1 = xk + K(zk+1 − Hxk )
        X = X.plus(K.times((Xk.times(H)).minus(H.times(X))));

        // update P
        P = (I.minus(K.times(H))).times(P);
        
        return X;
    }
}

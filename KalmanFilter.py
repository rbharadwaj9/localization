import numpy as np

def KalmanFilter(mu, Sigma, z, u, A, B, C, Q, R):
    
    #prediction step    
    mu_new = A*mu + B*u
    Sigma_new = A*Sigma*A.T + R

    #correction step
    K = Sigma_new * C.T * np.linalg.inv(C * Sigma_new * C.T + Q)
    mu_new = mu_new + K * (z - C * mu_new)
    Sigma_new = (np.eye(K.shape[0]) - K * C) * Sigma_new    

    return mu_new, Sigma_new

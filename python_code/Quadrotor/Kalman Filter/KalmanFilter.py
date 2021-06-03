import numpy as np
from numpy.random import randn
from numpy.linalg import inv, det
import math
import matplotlib.pyplot as plt

class KalmanFilter(object):
    def __init__(self):
        pass

    def predict(self, X, P, A, Q, B, U):
        X = np.dot(A, X) + np.dot(B, U)
        P = np.dot(A, np.dot(P, A.T)) + Q
        return(X, P)

    def update(self, X, P, Y, C, R):
        Xtmin = np.dot(C, X) # blm ditambah noise sensor Zt
        CPkCt = np.dot(C, np.dot(P, C.T)) + R
        K = np.dot(P, np.dot(C.T, inv(CPkCt)))
        X = X + np.dot(K, (Y-Xtmin))
        P = P - np.dot(K, np.dot(CPkCt, K.T))
        LH = self.gauss_pdf(Y, Xtmin, CPkCt)
        return (X, P, K, Xtmin, CPkCt, LH)

    def gauss_pdf(self, X, M, S):
        if M.shape[1] == 1:
            DX = X - np.tile(M, X.shape[1])
            E = 0.5 * np.sum(DX * (np.dot(inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(det(S))
            P = np.exp(-E)
        elif X.shape()[1] == 1:
            DX = np.tile(X, M.shape[1]) - M
            E = 0.5 * np.sum(DX * (np.dot(inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(det(S))
            P = np.exp(-E)
        else:
            DX = X-M
            E = 0.5 * np.dot(DX.T, np.dot(inv(S), DX))
            E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(det(S))
            P = np.exp(-E)
            return (P[0], E[0])
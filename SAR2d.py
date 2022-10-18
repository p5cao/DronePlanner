import numpy as np
import matplotlib.pyplot as plt

class SAR2d:
    def __init__(self, h, CSP, S, dN, dE) -> None:
        self.h = h
        self.CSP = CSP
        self.S = S
        self.dN = dN
        self.dE = dE

    def parallelSearch(self, h, CSP, S, dN, dE):
        px, py = CSP[0], CSP[1]
        i = 1
        path = [CSP]
        while py <= CSP[0] + dN:
            if i%2==0:
                py = py + S
            elif (i-1)%4==0:
                px = CSP[1] + dE - S/2
            else:
                px = CSP[1] + S/2
            if py > CSP[0] + dN:
                break
            path = np.vstack((path, np.array([px, py, h])))
            i += 1
        return path
    
    # def plot



         
 
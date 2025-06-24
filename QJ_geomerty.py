import numpy as np

class QJgeomerty:
    def __init__(self, width):
        self.theta = 0
        self.phi = 0

        self.da = 0
        self.db = 0
        self.dc = 0

        self.width = width

    def read_angle(self):
        return self.theta, self.phi
    
    def set_angle(self, theta, phi):
        self.theta = theta
        self.phi = phi

        self.da = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi)
        self.db = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi + (2/3 * np.pi))
        self.dc = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi+ (4/3 * np.pi))


        return self.da, self.db, self.dc
    
    def read_tendons_d(self):
        return self.da, self.db, self.dc
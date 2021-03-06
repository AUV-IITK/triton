#!/usr/bin/python
import numpy


class PIDRegulator:
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d
       

        self.integral = 0
        self.prev_err = 0

    def __str__(self):
        msg = 'PID controller:'
        msg += '\n\tp=%f' % self.p
        msg += '\n\ti=%f' % self.i
        msg += '\n\td=%f' % self.d
        
        return msg

    def regulate(self, err, dt):
        derr_dt = 0.0
        
        derr_dt = (err - self.prev_err)/dt
        self.integral += 0.5*(err + self.prev_err)*dt

        u = self.p*err + self.d*derr_dt + self.i*self.integral

        self.prev_err = err

        return u

#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-


class PID_incremental:
    def __init__(self, p, i, d, i_a, o_t):
        self.update_param(p, i, d, i_a, o_t)
        self.reset()

    def update_param(self, p, i, d, i_a, o_t):
        self.kp = p
        self.ki = i
        self.kd = d
        self.i_a = i_a
        self.o_t = o_t
        self.reset()

    def reset(self):
        self.err = 0.0
        self.e_pre_1 = 0.0
        self.e_pre_2 = 0.0
        self.u_out = 0.0

    def update(self, target, now):
        self.err = target - now
        
        delta_u = self.kp * (self.err - self.e_pre_1) + \
                  self.kd * (self.err - 2 * self.e_pre_1 + self.e_pre_2)
        
        if abs(self.err) < self.i_a:
            delta_u += self.ki * self.err
        
        self.e_pre_2 = self.e_pre_1
        self.e_pre_1 = self.err
        
        self.u_out += delta_u
        
        if abs(self.u_out) > self.o_t:
            u_out_g = self.o_t if self.u_out >= 0 else -self.o_t
        else:
            u_out_g = self.u_out
            
        return u_out_g
    
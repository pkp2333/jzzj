"""Cross-process C-stage execution ownership (separate from the arm I2C lease)."""
import os


class ExecutionLease:
    def __init__(self,path='/tmp/jzzj_stage_c_execution.lock'):
        import fcntl
        self.fd=os.open(path,os.O_CREAT|os.O_RDWR,0o600)
        try:fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError:
            self.close();raise RuntimeError('another C-stage entry owns execution')

    def close(self):
        if getattr(self,'fd',-1)>=0:os.close(self.fd);self.fd=-1

    def __del__(self):self.close()

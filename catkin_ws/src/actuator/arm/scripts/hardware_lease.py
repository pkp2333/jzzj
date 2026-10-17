"""Linux process-lifetime hardware ownership; never instantiated by offline tests."""
class HardwareLease:
    def __init__(self,side):
        import fcntl
        import os
        if side not in ('left','right'):raise ValueError('invalid arm side')
        self.fd=os.open('/tmp/jzzj_arm_'+side+'.lock',os.O_CREAT|os.O_RDWR,0o600)
        try:fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError:
            os.close(self.fd)
            self.fd=-1
            raise RuntimeError('another process owns the '+side+' arm hardware')
    def __del__(self):
        import os
        try:
            if self.fd>=0:os.close(self.fd)
        except (AttributeError,OSError):pass

#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, fcntl, math, os, select, signal, subprocess, sys, time
from datetime import datetime
from pathlib import Path
import serial

AGV_ROOT = Path(os.environ.get('AGV_ROOT', str(Path.home() / 'agv'))).expanduser().resolve()
OUT_DEFAULT = AGV_ROOT / 'data/navigasi/IMU_MAG_CAL'
G = 9.80665

def wrap360(v): return float(v) % 360.0
def heading_deg(x, y): return wrap360(math.degrees(math.atan2(y, x)))
def i16le(b, o): return int.from_bytes(b[o:o+2], 'little', signed=True)
def iso_now(): return datetime.now().astimezone().isoformat(timespec='milliseconds')

def pids_matching(patterns):
    out = subprocess.run(['ps','-eo','pid=,ppid=,cmd='], text=True, capture_output=True).stdout
    hits=[]
    for line in out.splitlines():
        s=line.strip()
        if not s: continue
        parts=s.split(None,2)
        if len(parts)<3: continue
        pid,ppid,cmd=int(parts[0]),int(parts[1]),parts[2]
        if pid==os.getpid(): continue
        if any(p in cmd for p in patterns): hits.append((pid,ppid,cmd))
    return hits
def stop_ros_serial_users(neo_port, imu_port):
    patterns=[
        'stmf4_hmi_bridge', 'data_imu_node', '/imu_node',
        'navigation imu.launch.py', 'stmf4 launch', 'stmf4_hmi',
    ]
    hits=pids_matching(patterns)
    # Stop parent launch first, then child node, to prevent respawn races.
    targets={pid for pid,_,_ in hits}
    for _,ppid,_ in hits:
        if ppid>1:
            try:
                pcmd=subprocess.run(['ps','-p',str(ppid),'-o','cmd='],text=True,capture_output=True).stdout
                if 'ros2 launch' in pcmd: targets.add(ppid)
            except Exception: pass
    for pid in sorted(targets):
        try: os.kill(pid, signal.SIGTERM)
        except ProcessLookupError: pass
    deadline=time.monotonic()+2.0
    while time.monotonic()<deadline and targets:
        targets={p for p in targets if Path(f'/proc/{p}').exists()}
        if targets: time.sleep(0.05)
    for pid in targets:
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass
    # Port-owner fallback: exclusive logger means no competing serial owner may survive.
    for dev in (neo_port, imu_port):
        subprocess.run(['fuser','-k',dev], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.25)

class YawLogger:
    def __init__(self, args):
        self.a=args; self.running=True; self.start=datetime.now().astimezone(); self.t0=time.monotonic()
        self.out=Path(args.output_dir).expanduser().resolve(); self.out.mkdir(parents=True, exist_ok=True)
        self.lock=open('/tmp/agv_imu_mag_yaw_logger.lock','w')
        try: fcntl.flock(self.lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: raise SystemExit('imu_mag_yaw_logger sudah berjalan')
        stop_ros_serial_users(args.neo_port,args.imu_port)
        self.neo=serial.Serial(args.neo_port,args.neo_baud,timeout=0,exclusive=True)
        self.imu=serial.Serial(args.imu_port,args.imu_baud,timeout=0,exclusive=True)
        self.active=self.out/f'.active_{os.getpid()}.csv'; self.f=self.active.open('w',newline='',buffering=1)
        self.cols=self.columns(); self.w=csv.DictWriter(self.f,fieldnames=self.cols); self.w.writeheader()
        self.nb=bytearray(); self.ib=bytearray(); self.rows=0; self.bad_checksum=0
        self.rm=[float('nan')]*3; self.rm_yaw=float('nan'); self.neo_node=''; self.neo_raw=''
        self.acc_raw=[None]*3; self.acc=[float('nan')]*3
        self.gyro_raw=[None]*3; self.gyro_dps=[float('nan')]*3; self.gyro_rps=[float('nan')]*3
        self.angle_raw=[None]*3; self.rpy=[float('nan')]*3
        self.mag_raw=[None]*3; self.imu_mag_yaw=float('nan')
        self.gyro_int=0.0; self.last_gyro_t=None; self.imu_raw=''; self.imu_checksum=''
        self.counters={'rm':0,'acc':0,'gyro':0,'angle':0,'mag':0}
        self.last_display=0.0
        self.recover_sent=False
        self.recover_due=time.monotonic()+3.0
        try:
            self.neo.write(b'PING\nNEO:STATUS\n')
            self.neo.flush()
        except Exception:
            pass

    @staticmethod
    def columns():
        return [
            'wall_time_iso','elapsed_s','source','packet_type',
            'rm_raw_x_ut','rm_raw_y_ut','rm_raw_z_ut','rm_yaw_deg','neo_node_id','neo_raw_line',
            'imu_acc_raw_x','imu_acc_raw_y','imu_acc_raw_z','imu_acc_x_mps2','imu_acc_y_mps2','imu_acc_z_mps2','imu_acc_norm_mps2',
            'imu_gyro_raw_x','imu_gyro_raw_y','imu_gyro_raw_z','imu_gyro_x_dps','imu_gyro_y_dps','imu_gyro_z_dps',
            'imu_gyro_x_rps','imu_gyro_y_rps','imu_gyro_z_rps','imu_gyro_integrated_yaw_deg',
            'imu_angle_raw_roll','imu_angle_raw_pitch','imu_angle_raw_yaw','imu_roll_deg','imu_pitch_deg','imu_accgyro_yaw_deg',
            'imu_mag_raw_x','imu_mag_raw_y','imu_mag_raw_z','imu_mag_yaw_deg',
            'imu_raw_frame_hex','imu_checksum_ok','rm_count','imu_acc_count','imu_gyro_count','imu_angle_count','imu_mag_count','bad_imu_checksum'
        ]

    def row(self, source, packet_type):
        an=math.sqrt(sum(v*v for v in self.acc)) if all(math.isfinite(v) for v in self.acc) else float('nan')
        return dict(
            wall_time_iso=iso_now(), elapsed_s=f'{time.monotonic()-self.t0:.6f}', source=source, packet_type=packet_type,
            rm_raw_x_ut=self.rm[0], rm_raw_y_ut=self.rm[1], rm_raw_z_ut=self.rm[2], rm_yaw_deg=self.rm_yaw,
            neo_node_id=self.neo_node, neo_raw_line=self.neo_raw,
            imu_acc_raw_x=self.acc_raw[0], imu_acc_raw_y=self.acc_raw[1], imu_acc_raw_z=self.acc_raw[2],
            imu_acc_x_mps2=self.acc[0], imu_acc_y_mps2=self.acc[1], imu_acc_z_mps2=self.acc[2], imu_acc_norm_mps2=an,
            imu_gyro_raw_x=self.gyro_raw[0], imu_gyro_raw_y=self.gyro_raw[1], imu_gyro_raw_z=self.gyro_raw[2],
            imu_gyro_x_dps=self.gyro_dps[0], imu_gyro_y_dps=self.gyro_dps[1], imu_gyro_z_dps=self.gyro_dps[2],
            imu_gyro_x_rps=self.gyro_rps[0], imu_gyro_y_rps=self.gyro_rps[1], imu_gyro_z_rps=self.gyro_rps[2],
            imu_gyro_integrated_yaw_deg=wrap360(self.gyro_int),
            imu_angle_raw_roll=self.angle_raw[0], imu_angle_raw_pitch=self.angle_raw[1], imu_angle_raw_yaw=self.angle_raw[2],
            imu_roll_deg=self.rpy[0], imu_pitch_deg=self.rpy[1], imu_accgyro_yaw_deg=self.rpy[2],
            imu_mag_raw_x=self.mag_raw[0], imu_mag_raw_y=self.mag_raw[1], imu_mag_raw_z=self.mag_raw[2], imu_mag_yaw_deg=self.imu_mag_yaw,
            imu_raw_frame_hex=self.imu_raw, imu_checksum_ok=self.imu_checksum,
            rm_count=self.counters['rm'], imu_acc_count=self.counters['acc'], imu_gyro_count=self.counters['gyro'],
            imu_angle_count=self.counters['angle'], imu_mag_count=self.counters['mag'], bad_imu_checksum=self.bad_checksum)
    def log(self, source, packet_type):
        self.w.writerow(self.row(source,packet_type)); self.rows+=1

    def parse_neo(self,line):
        self.neo_raw=line
        if line.startswith('SENS:MAGPRO:'):
            try:
                f=line.split(':',2)[2].split(','); self.neo_node=f[2]
                self.rm=list(map(float,(f[5],f[6],f[7]))); self.rm_yaw=heading_deg(self.rm[0],self.rm[1]); self.counters['rm']+=1
            except Exception: pass
            self.log('NEO3PRO','RM3100_MAG')
        elif line.startswith('SENS:NODE:'):
            try:self.neo_node=line.split(':',2)[2].split(',')[2]
            except Exception:pass
            self.log('NEO3PRO','NODE')
        elif line.startswith(('SENS:GNSSPRO:','SENS:GNSSCOV:','SENS:GNSSSTAT:','SENS:BARO:','SENS:TEMP:','SENS:HWPRO:','SENS:CANRX:','SENS:CANHEALTH:')):
            self.log('NEO3PRO',line.split(':',2)[1])

    def parse_imu(self,fr,now):
        self.imu_raw=fr.hex(' '); ok=(sum(fr[:10])&0xff)==fr[10]; self.imu_checksum=1 if ok else 0
        if not ok:
            self.bad_checksum+=1; self.log('YAHBOOM',f'BAD_CHECKSUM_0x{fr[1]:02X}'); return
        typ=fr[1]; v=[i16le(fr,i) for i in (2,4,6)]
        if typ==0x51:
            self.acc_raw=v; self.acc=[x/32768.0*self.a.accel_fsr_g*G for x in v]; self.counters['acc']+=1; self.log('YAHBOOM','ACC_0x51')
        elif typ==0x52:
            self.gyro_raw=v; self.gyro_dps=[x/32768.0*self.a.gyro_fsr_dps for x in v]; self.gyro_rps=[math.radians(x) for x in self.gyro_dps]
            if self.last_gyro_t is not None:
                dt=now-self.last_gyro_t
                if 0.0<dt<0.2:self.gyro_int+=self.gyro_dps[2]*dt
            self.last_gyro_t=now; self.counters['gyro']+=1; self.log('YAHBOOM','GYRO_0x52')
        elif typ==0x53:
            self.angle_raw=v; self.rpy=[x/32768.0*180.0 for x in v]; self.rpy[2]=wrap360(self.rpy[2]); self.counters['angle']+=1; self.log('YAHBOOM','ANGLE_0x53')
        elif typ==0x54:
            self.mag_raw=v; self.imu_mag_yaw=heading_deg(v[0],v[1]); self.counters['mag']+=1; self.log('YAHBOOM','MAG_0x54')
        elif 0x55<=typ<=0x59:
            self.log('YAHBOOM',f'RAW_0x{typ:02X}')

    def display(self):
        now=time.monotonic()
        if now-self.last_display<1.0/self.a.display_hz:return
        self.last_display=now
        def fmt(v):return '   ---   ' if not math.isfinite(v) else f'{v:8.2f}°'
        sys.stdout.write('\rRM3100: '+fmt(self.rm_yaw)+' | YAH MAG: '+fmt(self.imu_mag_yaw)+' | YAH ACC/GYRO: '+fmt(self.rpy[2]))
        sys.stdout.flush()
    def run(self):
        signal.signal(signal.SIGINT,lambda *_: setattr(self,'running',False))
        signal.signal(signal.SIGTERM,lambda *_: setattr(self,'running',False))
        while self.running:
            now=time.monotonic()
            ready,_,_=select.select([self.neo.fileno(),self.imu.fileno()],[],[],0.02)
            if self.neo.fileno() in ready:
                self.nb.extend(os.read(self.neo.fileno(),4096))
                while b'\n' in self.nb:
                    line,_,self.nb=self.nb.partition(b'\n')
                    s=line.decode('ascii','ignore').strip()
                    if s:self.parse_neo(s)
            if self.imu.fileno() in ready:
                self.ib.extend(os.read(self.imu.fileno(),4096))
                while len(self.ib)>=11:
                    if self.ib[0]!=0x55 or not (0x51<=self.ib[1]<=0x59):
                        del self.ib[0]; continue
                    fr=bytes(self.ib[:11])
                    if (sum(fr[:10])&0xff)!=fr[10]:
                        # Log bad candidate, then slide one byte to resync.
                        self.imu_raw=fr.hex(' '); self.imu_checksum=0; self.bad_checksum+=1
                        self.log('YAHBOOM',f'BAD_CHECKSUM_0x{fr[1]:02X}'); del self.ib[0]; continue
                    del self.ib[:11]; self.parse_imu(fr,now)
            if (not self.recover_sent and self.counters['rm']==0 and time.monotonic()>=self.recover_due):
                try:
                    self.neo.write(b'NEO:CAN:RECOVER\n')
                    self.neo.flush()
                except Exception:
                    pass
                self.recover_sent=True
            self.display()
        self.finish()

    def finish(self):
        try:self.neo.close()
        except Exception:pass
        try:self.imu.close()
        except Exception:pass
        try:self.f.flush(); os.fsync(self.f.fileno()); self.f.close()
        except Exception:pass
        stop=datetime.now().astimezone()
        name=f'IMU_MAG_CAL_{self.start.strftime("%Y%m%d_%H%M%S")}_to_{stop.strftime("%Y%m%d_%H%M%S")}.csv'
        final=self.out/name
        os.replace(self.active,final); self.final_path=final
        sys.stdout.write('\n')
        sys.stdout.flush()
        print(f'SAVED: {final}')


def main():
    ap=argparse.ArgumentParser(description='Exclusive standalone RM3100/Yahboom yaw monitor and raw CSV logger')
    ap.add_argument('--neo-port',default='/dev/ttyACM0'); ap.add_argument('--neo-baud',type=int,default=1000000)
    ap.add_argument('--imu-port',default='/dev/ttyUSB0'); ap.add_argument('--imu-baud',type=int,default=921600)
    ap.add_argument('--output-dir',default=str(OUT_DEFAULT)); ap.add_argument('--display-hz',type=float,default=10.0)
    ap.add_argument('--accel-fsr-g',type=float,default=16.0); ap.add_argument('--gyro-fsr-dps',type=float,default=2000.0)
    a=ap.parse_args()
    try:
        lg=YawLogger(a); lg.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'ERROR: {e}',file=sys.stderr); raise SystemExit(2)

if __name__=='__main__':main()

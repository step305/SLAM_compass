import os
import json
import threading
import time
import signal
from subprocess import check_output
import socket
import base64
import numpy as np
import cv2


FIFO = '/home/step305/SLAM_FIFO.tmp'
#FIFO = '/home/sergey/SLAM_FIFO.tmp'


earth_meas = [0, 0]
TIME_MEASURE = 30
TIME_SLEEP = 15

data_ready = threading.Event()
quit_prog = threading.Event()


def read_thread():
    global earth_meas
    global data_ready
    cnt = 0
    max_cnt = 10
    heading_sum = 0
    roll_sum = 0
    pitch_sum = 0
    bw_sum = [0, 0, 0]
    crh_sum = 0

    while not quit_prog.is_set():
        try:
            with open(FIFO) as fifo:
                for line in fifo:
                    if quit_prog.is_set():
                        pid = int(check_output(["pidof", 'SLAM_NANO']))
                        os.kill(pid, signal.SIGINT)
                        time.sleep(1)
                        break
                    packet = json.loads(line)
                    if cnt == max_cnt:
                        earth_meas = [heading_sum/cnt, crh_sum/cnt]
                        data_ready.set()
                        heading_sum = 0
                        roll_sum = 0
                        pitch_sum = 0
                        bw_sum = [0, 0, 0]
                        crh_sum = 0
                        cnt = 0
                    else:
                        cnt = cnt + 1
                        heading_sum += packet['yaw']
                        roll_sum += packet['roll']
                        pitch_sum += packet['pitch']
                        bw_sum[0] += packet['bw'][0]
                        bw_sum[1] += packet['bw'][1]
                        bw_sum[2] += packet['bw'][2]
                        crh_sum += packet['adc']
                        buf_decode = base64.b64decode(packet['frame'])
                        jpg = np.fromstring(buf_decode, np.uint8).tobytes()
        except Exception as e:
            print()
            print(e)
            print('Done!')
            break


if __name__ == '__main__':
    print('Starting')
    #time.sleep(TIME_SLEEP)
    reader = threading.Thread(target=read_thread, args=())
    reader.start()
    earth_meas_hist = []
    print_cnt = 0
    wait_line = '.' * 20
    time.sleep(5)

    while not quit_prog.is_set():
        com = input('rotate compass and press Enter (to stop - press q and Enter)')
        if com == 'q':
            quit_prog.set()
            pid = int(check_output(["pidof", 'SLAM_NANO']))
            os.kill(pid, signal.SIGINT)
            time.sleep(1)
            break
        print('Wait...')
        time.sleep(TIME_SLEEP)
        t0 = time.time()
        eart_meas_sum = [0, 0]
        cnt = 0
        while time.time() - t0 < TIME_MEASURE:
            if data_ready.is_set():
                print(wait_line[:print_cnt] + '*' + wait_line[print_cnt + 1:], end='\r')
                if print_cnt < 20:
                    print_cnt += 1
                else:
                    print_cnt = 0
                data_ready.clear()
                eart_meas_sum[0] += earth_meas[0]
                eart_meas_sum[1] += earth_meas[1]
                cnt += 1
        print()
        eart_meas_sum[0] = eart_meas_sum[0]/cnt
        eart_meas_sum[1] = eart_meas_sum[1]/cnt
        earth_meas_hist.append(eart_meas_sum)

        print('history:')
        earth_meas_hist.sort(key=lambda x: x[0])
        for meas in earth_meas_hist:
            print('heading = {:.2f}deg -> CRH = {:.2f}dph'.format(meas[0], meas[1]))
    reader.join()

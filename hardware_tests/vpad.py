#!/usr/bin/env python3
"""Virtual gamepad for MiSTer via /dev/uinput.

Clones the identity of the user's wired pad (045e:028e "Microsoft X-Box
360 pad") so MiSTer applies the existing mapping in
/media/fat/config/inputs/input_045e_028e_v3.map to it with no setup.

Usage:  vpad.py [--hold S] [--gap S] [--settle S] btn [btn ...]
        vpad.py --idle S            # just create the device and hold it

Buttons: a b x y tl tr select start mode  up down left right (dpad hat)
Also accepts a raw evdev key code as a number, e.g. 308.
"""
import fcntl, struct, sys, time

UI_SET_EVBIT  = 0x40045564
UI_SET_KEYBIT = 0x40045565
UI_SET_ABSBIT = 0x40045567
UI_DEV_CREATE  = 0x00005501
UI_DEV_DESTROY = 0x00005502
EV_SYN, EV_KEY, EV_ABS = 0, 1, 3
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0, 1, 2, 3, 4, 5
ABS_HAT0X, ABS_HAT0Y = 16, 17

KEYS = dict(a=304, b=305, x=307, y=308, tl=310, tr=311,
            select=314, start=315, mode=316, thumbl=317, thumbr=318)
HATS = dict(left=(ABS_HAT0X, -1), right=(ABS_HAT0X, 1),
            up=(ABS_HAT0Y, -1), down=(ABS_HAT0Y, 1))


def emit(f, etype, code, value):
    f.write(struct.pack('llHHi', 0, 0, etype, code, value))
    f.flush()


def syn(f):
    emit(f, EV_SYN, 0, 0)


def main():
    args = sys.argv[1:]
    hold, gap, settle, idle = 0.20, 0.45, 2.0, None
    presses = []
    i = 0
    while i < len(args):
        a = args[i]
        if a == '--hold':
            hold = float(args[i + 1]); i += 2
        elif a == '--gap':
            gap = float(args[i + 1]); i += 2
        elif a == '--settle':
            settle = float(args[i + 1]); i += 2
        elif a == '--idle':
            idle = float(args[i + 1]); i += 2
        else:
            presses.append(a); i += 1

    f = open('/dev/uinput', 'wb')
    fd = f.fileno()
    fcntl.ioctl(fd, UI_SET_EVBIT, EV_KEY)
    fcntl.ioctl(fd, UI_SET_EVBIT, EV_ABS)
    for code in KEYS.values():
        fcntl.ioctl(fd, UI_SET_KEYBIT, code)
    for code in (ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ,
                 ABS_HAT0X, ABS_HAT0Y):
        fcntl.ioctl(fd, UI_SET_ABSBIT, code)

    # struct uinput_user_dev: name[80], input_id{bus,vid,pid,ver},
    # ff_effects_max, absmax[64], absmin[64], absfuzz[64], absflat[64]
    absmax = [0] * 64
    absmin = [0] * 64
    for ax in (ABS_X, ABS_Y, ABS_RX, ABS_RY):
        absmax[ax], absmin[ax] = 32767, -32768
    for ax in (ABS_Z, ABS_RZ):
        absmax[ax], absmin[ax] = 255, 0
    for ax in (ABS_HAT0X, ABS_HAT0Y):
        absmax[ax], absmin[ax] = 1, -1
    dev = struct.pack('80sHHHHi', b'Microsoft X-Box 360 pad',
                      0x0003, 0x045e, 0x028e, 0x0114, 0)
    dev += struct.pack('64i', *absmax) + struct.pack('64i', *absmin)
    dev += struct.pack('64i', *([0] * 64)) + struct.pack('64i', *([0] * 64))
    f.write(dev)
    f.flush()
    fcntl.ioctl(fd, UI_DEV_CREATE)

    time.sleep(settle)                 # let MiSTer pick the device up
    emit(f, EV_ABS, ABS_X, 0)
    emit(f, EV_ABS, ABS_Y, 0)
    syn(f)

    if idle is not None:
        time.sleep(idle)

    for name in presses:
        if '.' in name:                # bare float token = extra sleep
            time.sleep(float(name))
            continue
        if name in HATS:
            axis, value = HATS[name]
            emit(f, EV_ABS, axis, value); syn(f)
            time.sleep(hold)
            emit(f, EV_ABS, axis, 0); syn(f)
        else:
            code = KEYS.get(name) or int(name)
            emit(f, EV_KEY, code, 1); syn(f)
            time.sleep(hold)
            emit(f, EV_KEY, code, 0); syn(f)
        print('pressed', name)
        time.sleep(gap)

    time.sleep(0.5)
    fcntl.ioctl(fd, UI_DEV_DESTROY)
    f.close()


if __name__ == '__main__':
    main()

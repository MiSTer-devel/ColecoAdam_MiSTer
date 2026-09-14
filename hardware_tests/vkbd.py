#!/usr/bin/env python3
"""Virtual keyboard for MiSTer via /dev/uinput.

Creates a generic USB keyboard, waits for MiSTer's inotify to notice it,
then sends the requested keys as clean press/release pairs.

Usage: vkbd.py [--settle S] [--hold S] [--gap S] KEY [KEY ...]
KEY is an evdev key name from the table below, or a raw numeric code.
"""
import fcntl, struct, sys, time

UI_SET_EVBIT  = 0x40045564
UI_SET_KEYBIT = 0x40045565
UI_DEV_CREATE  = 0x00005501
UI_DEV_DESTROY = 0x00005502
EV_SYN, EV_KEY = 0, 1

NAMES = {
    'esc':1,'1':2,'2':3,'3':4,'4':5,'5':6,'6':7,'7':8,'8':9,'9':10,'0':11,
    'minus':12,'equal':13,'backspace':14,'tab':15,
    'q':16,'w':17,'e':18,'r':19,'t':20,'y':21,'u':22,'i':23,'o':24,'p':25,
    'lbrace':26,'rbrace':27,'enter':28,'lctrl':29,
    'a':30,'s':31,'d':32,'f':33,'g':34,'h':35,'j':36,'k':37,'l':38,
    'semicolon':39,'apostrophe':40,'grave':41,'lshift':42,'backslash':43,
    'z':44,'x':45,'c':46,'v':47,'b':48,'n':49,'m':50,
    'comma':51,'dot':52,'slash':53,'rshift':54,'kpasterisk':55,
    'lalt':56,'space':57,'capslock':58,
    'f1':59,'f2':60,'f3':61,'f4':62,'f5':63,'f6':64,'f7':65,'f8':66,
    'f9':67,'f10':68,'f11':87,'f12':88,
    'up':103,'left':105,'right':106,'down':108,
}

def emit(f, etype, code, value):
    f.write(struct.pack('llHHi', 0, 0, etype, code, value)); f.flush()

def main():
    args = sys.argv[1:]
    settle, hold, gap = 3.0, 0.08, 0.9
    keys = []
    i = 0
    while i < len(args):
        a = args[i]
        if a == '--settle': settle = float(args[i+1]); i += 2
        elif a == '--hold': hold = float(args[i+1]); i += 2
        elif a == '--gap':  gap  = float(args[i+1]); i += 2
        else: keys.append(a); i += 1

    f = open('/dev/uinput', 'wb')
    fd = f.fileno()
    fcntl.ioctl(fd, UI_SET_EVBIT, EV_KEY)
    for c in range(1, 128):
        fcntl.ioctl(fd, UI_SET_KEYBIT, c)
    # uinput_user_dev: name[80], id{bustype,vendor,product,version}, ff_effects, abs arrays
    name = b'MiSTer Test Keyboard'.ljust(80, b'\0')
    dev = name + struct.pack('HHHH', 0x03, 0x046d, 0xc31c, 0x0110) + struct.pack('i', 0)
    dev += b'\0' * (4 * 64 * 4)
    f.write(dev); f.flush()
    fcntl.ioctl(fd, UI_DEV_CREATE)
    time.sleep(settle)

    for k in keys:
        code = int(k) if k.isdigit() else NAMES[k.lower()]
        emit(f, EV_KEY, code, 1); emit(f, EV_SYN, 0, 0)
        time.sleep(hold)
        emit(f, EV_KEY, code, 0); emit(f, EV_SYN, 0, 0)
        print('sent %s (%d)' % (k, code))
        time.sleep(gap)

    time.sleep(0.5)
    fcntl.ioctl(fd, UI_DEV_DESTROY)
    f.close()

main()

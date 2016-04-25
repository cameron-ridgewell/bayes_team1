#!/usr/bin/env python
import sys,tty,termios
import urllib2
from time import sleep

class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
        inkey = _Getch()
        while(exit):
                k=inkey()
                if k!='':break
        if k=='\x1b[A':
                print "up"
		return 0;
        elif k=='\x1b[B':
                print "down"
		return 2;
        elif k=='\x1b[C':
                print "right"
		return 6;
        elif k=='\x1b[D':
                print "left"
		return 4;
        else:
		print "press key again to exit"
		return -1;

def sendCommand(comm):
	response = urllib2.urlopen('http://192.168.1.41:81/decoder_control.cgi?loginuse=admin&loginpas=12345&command='+str(comm)+'&onestep=0&14434782369140.2543632062152028&_=1443478236914');
	sleep(0.1);
	urllib2.urlopen('http://192.168.1.41:81/decoder_control.cgi?loginuse=admin&loginpas=12345&command=7&onestep=1&14434782369140.2543632062152028&_=1443478236914');

def main():
	last = get();
        while(last >= 0):
		last = get()
		if last >= 0:
			sendCommand(last)

if __name__=='__main__':
        main()

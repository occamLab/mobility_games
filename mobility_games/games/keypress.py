import tty
import sys
import termios

orig_settings = termios.tcgetattr(sys.stdin)

x = 0
while x != chr(27): # ESC
    x=sys.stdin.read(1)
    print("keypress")
    break

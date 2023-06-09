import time
import serial

f=open('motors.txt','w')

ser=serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=2
)

def set_serial(a):
	ser=serial.Serial(

		port='/dev/ttyUSB0',
		baudrate=9600,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		bytesize=serial.EIGHTBITS,
		timeout=1
	)

def new_read(a):
	ser.write(a)
	s=ser.read()
	m=int(hex(ord(s)),16)
	return m

def read(a):
	x=[]
	ser.write('a')
	s=ser.read()
	m=int(hex(ord(s)),16)
	x.append(m)
	ser.write('b')
	s=ser.read()
	m=int(hex(ord(s)),16)
	x.append(m)
	ser.write('c')
        s=ser.read()
        m=int(hex(ord(s)),16)
	x.append(m)
	ser.write('d')
        s=ser.read()
        m=int(hex(ord(s)),16)
	x.append(m)
	ser.write('e')
        s=ser.read()
        m=int(hex(ord(s)),16)
	x.append(m)
	return x

def read_sharp(a):
        val=[]
        ser.write(a)
        #slen=ser.read(10)
        a=ser.read()
	#return ord(a)
	b=ser.read()
	#return ord(b)
	return ord(a)*256+ord(b)
	c=a+b*256
	return c


'''def read_sharp(a):
	val=[]
	ser.write('S')
	slen=ser.read(10)
	return slen
	newv=int(slen)
	ser.write('1')
	s=ser.read()
	return s
	for i in range(1,newv+1):
		ser.write(chr(i))
		s=ser.read()
		val.append(s)
	return val
'''
def move_bot(a):  #a is the speed of both the motors and should be in the format : m100#100!
	ser.write(str(a))
	f.write(a)
	return a

#ser.write("120.5#100.5!")
'''while True:
	s=ser.read()
	#hexval=s.decode('hex')
	#print hex(ord(s))
	print int(hex(ord(s)),16)'''
#ser.write(".")
'''while True:
	s=ser.read()
	print s'''
'''while True:
	for s in ser.read():
		print s
'''
#ser.close()
'''time.sleep(2)
ser.write("#")
time.sleep(1)
ser.write("\x35")'''

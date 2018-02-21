


import time
import threading 
import signal


def test(name, delay, run_event):

	while run_event.is_set():
		print name
		#time.sleep(1)

def handler(signum, frame):
	time.sleep(1)
	print 'ignoring ctrl z'


run_event = threading.Event()
run_event.set()
t1 = threading.Thread(target=test, args = ('pls', 1, run_event))
t1.start()


try:
	while 1:
		
		time.sleep(0.1)
except KeyboardInterrupt:
	print 'closing threads'
	run_event.clear()
	print 'sent clear'
	t1.join()
	print 'we be done'

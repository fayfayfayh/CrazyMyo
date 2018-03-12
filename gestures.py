
import Queue


class Gesture:

	def __init__(self):
		self.queue = Queue.Queue() # FIFO

	def add_gesture(self, g):
		
		print g
		self.queue.put(g, block=False)
		

	def get_gesture(self):
		try:
			g = self.queue.get(block=False)
		except Queue.Empty:
			g = None
		if g is not None:
			print g
		return g
		
		
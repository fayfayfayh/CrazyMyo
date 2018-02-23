
import Queue


class Gesture:

	def __init__(self):
		self.queue = Queue.Queue() # FIFO

	def add_gesture(self,id):
		self.queue.put(id)

	def get_gesture(self):
		try:
			g = self.queue.get(block=False)
		except Queue.Empty:
			g = None

		return g
		
		
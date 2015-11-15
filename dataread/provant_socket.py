__author__ = 'Patrick'
import socket
import sys
#a simple test of a class that listen a socket server
#need an update with vinicius
class ProvantSocket:
	def __init__(self, parent = None):
		# Create a TCP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server_address = ('localhost', 666)
		self.data=[0]*512

	def connect(self,address,door):
		server_address = (address, door)
		print >>sys.stderr, 'connecting to %s port %s' % server_address
		self.sock.connect(server_address)

	def read(self):
		exist=True
		while(exist):
			self.data = self.sock.recv(1)
			if (self.data):
				 sys.stdout.write(self.data)
			else:
				exist=False

if __name__ == '__main__':
	mysocket = ProvantSocket()
	mysocket.connect('localhost',10000)
	mysocket.read()
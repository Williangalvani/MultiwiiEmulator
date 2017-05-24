import ping, socket, sys, Tkinter, tkMessageBox, time
from time import sleep
from Tkinter import *
import datetime
import random
import matplotlib.pyplot as plt

class Application(Frame):
	
	def mok(self):
		self.mtext = self.ment.get()
		return

	def LeUpdate(self):
		self.window.update()

	def createVariables(self):
		
		self.ment       = StringVar()
		self.statustext = StringVar()
		self.timertext  = StringVar()

		self.ment.set(u"www.google.com")
		self.mtext      = "www.google.com"

	def createWidgets(self):
		
		self.mlabel      = Label(self,text="Bote o endereco:").grid(row=0,column=0)
		self.mEntry      = Entry(self,textvariable=self.ment).grid(row=0,column=1)
		self.mbutton     = Button(self,text="OK!",command = self.mok).grid(row=1,column=0)

		self.statusLabel = Label(self,textvariable=self.statustext).grid(row=1,column=1)
		self.label1      = Label(self,textvariable=self.timertext,fg="black").grid(row=0,column=2)

	def __init__(self):
		self.window = Tk()
		self.window.title("PyPing")
		self.window.geometry("+200+200")

		Frame.__init__(self,self.window)
		self.pack()
		self.createVariables()
		self.createWidgets()

class LePlot():
	def LeDraw(self):
		plt.plot(self.x,self.y)
		#beautify the x-labels
		plt.gcf().autofmt_xdate()
		plt.ion()
		plt.show()
	def LeShow(self):
		plt.draw()	

	def __init__(self):
		self.x=list()
		self.y=list()

#Plot = LePlot(plt)
app  = Application()

#Plot.LeDraw()
while(1):
	app.LeUpdate()
	time.sleep(0.1)
	try:
		timer = ping.do_one(app.mtext, 3)
		timer = timer*1000
		app.timertext.set(" %.2f ms" % timer)
		app.statustext.set("=D")
	except socket.error, e:
		app.statustext.set(e)
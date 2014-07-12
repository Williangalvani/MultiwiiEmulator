__author__ = 'Will'

string = """armed
level
horizon
baro
mag
rth
gpshold
"""
counter = 0
for i in string.replace('\n'," "):

        print "buffer[{0}] = to_index('{1}');".format(counter,i.upper())
        counter+=1

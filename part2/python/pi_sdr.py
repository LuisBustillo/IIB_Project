from pylab import *
from rtlsdr import *

server = RtlSdrTcpServer(hostname='192.168.102.210', port=55366)
server.run_forever()
# Will listen for clients until Ctrl-C is pressed
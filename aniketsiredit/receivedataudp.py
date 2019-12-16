import socket
soc=[]
for i in range(1,101):
	soc.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
uavids=[12,23,4,5,6,44,5,6,7,8]
n=10
a=[]


for i in uavids:
	i=str(i)
	for j in range(n):
		j=j+5001
	
		address=(i,j)
		a.append(address)
for i in a:
	j=0
	soc[j].bind(i)
	j+=1

"""
import socket

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
address1 = ("192.168.4.212", 5002)
address2 = ("192.168.4.211", 5001)
sock1.bind(address1)
sock2.bind(address2)

t=0
n=0
while True:
	t1 = time.time()
	data1,addr1 = sock1.recvfrom(64)
	data2,addr2 = sock2.recvfrom(64)

	datalist1 = data1.strip("/n").split(" ")
	datalist1 = list(map(float,datalist1))
	datalist2 = data2.strip("/n").split(" ")
	datalist2 = list(map(float,datalist2))

	t2 = time.time()
	t = (t2-t1) + t
	n=n+1

	if t>1:
		print n
		n=0
		t=0
t=0
n=0
while True:
	t1 = time.time()
	data1,addr1 = sock1.recvfrom(64)
	data2,addr2 = sock2.recvfrom(64)

	datalist1 = data1.strip("/n").split(" ")
	datalist1 = list(map(float,datalist1))
	datalist2 = data2.strip("/n").split(" ")
	datalist2 = list(map(float,datalist2))

	t2 = time.time()
	t = (t2-t1) + t
	n=n+1

	if t>1:
		print() n)
		n=0+
		t=0
"""
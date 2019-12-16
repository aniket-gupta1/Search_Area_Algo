import socket
import struct
from dronekit import connect
import time
import multiprocessing
import json
import sys


Id = int(sys.argv[1])

class multicast(object):
    def __init__(self):
        self.public_dict={}
        #make these variables private
        self.multicast_addr = "224.0.0.0"
        self.bind_addr = "0.0.0.0"
        self.port = 6000 + Id
        self.vehicle=None
        self.sleep_time= .1
        self.refresh_rate= 3
        self.packet_counter=0
        self.vehicle_connection_string = "127.0.0.1:" + str(14550 + Id*10)
        self.local_ip="127.0.0.1"
        self.local_port=11111 + Id



        try:
            #self.vehicle=connect(self.vehicle_connection_string)
            pass

        except Exception as err :
            self.vehicle=None
            print("EXCEPTION IN CONNECTING TO UAV "+str(err))

        self.main_thread()

    def create_sender_socket(self):
        print("creating sender socket")
        try:
            self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,socket.IPPROTO_UDP)
            # sock.settimeout(.2)

            ttl = struct.pack('b', 1)
            #configuring socket in os in multicast mode
            self.send_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
            return self.send_sock
        except Exception as err:
            print("create_sender_socket"+ str(err))
            return None

    def create_and_bind_receiver_socket(self):
        try:

            self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,socket.IPPROTO_UDP)
            #membership = socket.inet_aton(self.multicast_addr) + socket.inet_aton(self.bind_addr)

            self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            membership = socket.inet_aton(self.multicast_addr) + socket.inet_aton(self.bind_addr)

            #group = socket.inet_aton(self.multicast_addr)
            #mreq = struct.pack("4sl", group, socket.INADDR_ANY)
            self.recv_sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.bind_addr))

            self.recv_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
            #self.recv_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)


            self.recv_sock.bind((self.bind_addr, self.port))
            return  self.recv_sock
        except Exception as err :
            print("create_receiver_socket" + str(err))
            print(err)
            return  None

    def get_encoded_telemetry(self):

        if self.vehicle is not None:

            lat = self.vehicle.location.global_frame.lat
            lon = self.vehicle.location.global_frame.lon
            alti= self.vehicle.location.global_relative_frame.alt
            uav_id=int(self.vehicle.parameters['SYSID_THISMAV'])
            self.packet_counter+=1
            #data = str(uav_id)+" "+str(lat) + " " + str(lon)+ " "+str(alti)+" "+str(self.packet_counter)
            #message = data.encode("utf-8")
            message=[uav_id,lat,lon,alti,self.packet_counter]

            return message
        else:

            try:
                self.vehicle = connect(self.vehicle_connection_string)
                lat = self.vehicle.location.global_frame.lat
                lon = self.vehicle.location.global_frame.lon

                data = str(lat) + " " + str(lon)
                message = data.encode("utf-8")

                return message
            except Exception as err :
                self.vehicle = None
                print("Exception encountered in get_encoded_telemetry",str(err))
                return None


    def recv_packets(self,dt):
        temp = {}
        human_dict={}

        while True:



            try:
                temp=dt["GEOLOCATIONS"]
                message, address = self.recv_sock.recvfrom(256)


                #print("Received: "+ str(address) + " " + message.decode("utf-8"))
                #queue.put([message, address])
                #get packet entry time
                data = json.loads(message.decode('utf-8'))
                print(data)
                uav_id=data["SYSID"]
                temp[uav_id] = [time.time()] + data["GEOLOCATIONS"]
                # temp[address[0]]=[time.time()]+message

                # print("time",time.time()-dt[address[0]][1])
                # check if a data is in the dictionary for more than refresh sec

                # print(dt[address[0]][1]-time.time())
                temp = {key: data for (key, data) in temp.items() if time.time() - data[0] < self.refresh_rate}

                # print("Temp ", temp)
                dt["GEOLOCATIONS"] = temp


                if data["HUMANS"]==None:
                    continue
                if uav_id in human_dict.keys():
                    for tup in data["HUMANS"]:
                        human_dict[uav_id].add(tuple(tup))
                else:
                    human_dict[uav_id] = set()
                    for tup in data["HUMANS"]:
                        human_dict[uav_id].add(tuple(tup))

                dt["HUMANS"]=human_dict
                #print("IN recv ",dt )


            except Exception as err:
                print("recv_package"+str(err))
                self.recv_sock.close()
                self.create_and_bind_receiver_socket()



    def get_human_location(self):
        lat=65.015
        lon=77.11586
        confidence=.9
        drop_flag=0
        human_list=[tuple([lat,lon,confidence,drop_flag])]
        #initialize socket here to get human_location
        return human_list

    
    def send_packets(self):

        #Trying to connect to vehicle again
        self.vehicle=connect(self.vehicle_connection_string)
        #self.vehicle=vehicle
        print("CONNECTED TO VEHICLE")
        temp={"SYSID":0,"GEOLOCATIONS":[],"HUMANS":[],"PACKETNO":0}

        while True:

            #message1 = self.get_encoded_telemetry()
            try:

                uav_id, lat, lon, alti,packet_counter=self.get_encoded_telemetry()
            except:
                print("error in send_packet")
                continue
            message2=self.get_human_location()

            temp["SYSID"]=uav_id
            temp["GEOLOCATIONS"]=[uav_id,lat,lon,alti,packet_counter]
            temp["HUMANS"]=message2
            temp["PACKETNO"]=packet_counter

            print("SENT MESSAGE", temp)
            if temp is None:
                continue
            else:
                try:
                    app_json = json.dumps(temp).encode("UTF-8")
                    #local_sock.sendto(app_json, (self.local_ip, self.local_port))

                    self.send_sock.sendto(app_json, (self.multicast_addr, self.port))
                    #print("message_sent")
                except Exception as err:
                    #print("send_pack"+str(err))
                    self.send_sock.close()
                    self.create_sender_socket()

            time.sleep(self.sleep_time)

    

    def main_thread(self):

        self.send_sock=self.create_sender_socket()
        if self.send_sock is None:
            self.send_sock = self.create_sender_socket()


        self.recv_sock = self.create_and_bind_receiver_socket()
        if self.recv_sock is None:
            self.recv_sock=self.create_and_bind_receiver_socket()
        with multiprocessing.Manager() as manager:

            #que=multiprocessing.Queue()
            #shared_dict=manager.dict({"DICT":{}})
            shared_dict = manager.dict({"GEOLOCATIONS": {},"HUMANS":{}})

            send_process=multiprocessing.Process(target=self.send_packets)
            #send_process = multiprocessing.Process(target=self.recv_packets, args=[shared_dict])
            recv_process = multiprocessing.Process(target=self.recv_packets, args=[shared_dict])

            send_process.start()
            recv_process.start()
            

            local_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            while True:
                print("shared",shared_dict)
                temp=shared_dict["GEOLOCATIONS"].copy()
                print("main",[key for key in sorted(temp.keys())])
                [print(key, " :: ", temp[key]) for key in sorted(temp.keys())]

                #for key,data in shared_dict["DICT"]:
                #    temp[key]=data
                try:

                    app_json = json.dumps(temp, sort_keys=True).encode("UTF-8")
                    local_sock.sendto(app_json,(self.local_ip,self.local_port))
                    print("sent to local adress")
                    #print("main", str(self.dt))
                except Exception as err:
                    local_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    print("local_socket err in main thread" ,err)


                """
                if not que.empty():

                    print("main",str(que.get()))
                    continue
                else:
                    print("queue empty")
                    time.sleep(2)
                """
                time.sleep(.1)
if __name__=="__main__":

    multicast()

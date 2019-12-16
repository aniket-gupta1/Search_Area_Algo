import dronekit
from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from math import *
import threading
from math import radians, cos, sin, asin, sqrt,atan2
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LineString
from helper import *
from time import time
from SwarmBot import SwarmBot
import sys

alti = 5
TMAX = 500
sal=1000#int(input("enter the length of the seach area "))
saw=1000#int(input("enter the width of search area"))
fol=100#int(inpu("enter the length of fov"))
fow=100#int(input("enter the width of the fov"))
lat=28.763638#float(input("enter the latitude"))
lon=77.114030#float(input("enter the longitude"))
heading=0#input("Enter the heading angle of uavs(in degrees):")
p1,p2,p3,p4=rectangle(sal,saw,lat,lon,heading)
n=10#int(input("enter the no of uavs "))
x=bearing(p1[0],p1[1],p2[0],p2[1])
INITIALWAY=intialwaypoints(n,p1,p2,p3,p4,x)
finalwaypoints=finalwaypoins(n,p1,p2,p3,p4,x)

Id = int(sys.argv[1])
self_connection_string = '127.0.0.1:' + str(14551 + 10*Id)
local_address="127.0.0."+str(Id+1)
local_port=5001

## Creating n socket objects for sending the data
sending_sock = list()
for i in range(n):
    sending_sock.append(create_uav_ports((local_address,local_port + i)))

## Creating n socket objects to receive data from other UAVs
receiving_sock = list()
for i in range(n):
    address = '127.0.0.'+str(i+1)
    receiving_sock.append(create_and_bind_uav_ports((address, local_port + Id)))
GlobalUavData=[]
for i in range(1,n+1):
    GlobalUavData.append({i:{"GPS":[], "Humans":[], "G":0, "P":0, "bestloc":[], "gbestloc":[]}} )
detected_humans = []

uav = SwarmBot(self_connection_string)
uav.arm_and_takeoff(5)
uav.update_pos([finalwaypoints[uav.id-1][0], finalwaypoints[uav.id-1][1],5],5)
wstart=0.9
wend=0.4

timer = 0
#5loooopy=[[28.766507012972816, 77.11291207446766, 0.83], [28.76868516219936, 77.1120458244287, 0.88], [28.77060317577803, 77.11739619464721, 0.8], [28.76893697227221, 77.11173803816813, 0.78], [28.770603175810493, 77.1169652858881, 0.73], [28.76902690445293, 77.11147129007564, 0.82], [28.770998877314007, 77.11700632481754, 0.82], [28.770747067262796, 77.11702684428225, 0.82], [28.768649189370684, 77.11157388549583, 0.74], [28.770789931596337, 77.11099568899553, 0.72], [28.770735972250513, 77.11165231100438, 0.99], [28.769008917994515, 77.11182011450428, 0.98], [28.77046617578111, 77.11148815550217, 0.86], [28.770495257183768, 77.11741671411194, 0.73], [28.771142768749314, 77.11729359732361, 0.91], [28.770987782327982, 77.11128296112439, 0.9], [28.770502148674723, 77.11109828618439, 0.75], [28.77056720295554, 77.11684216909978, 0.95], [28.77108880946069, 77.117191, 0.86], [28.766237216484864, 77.1129941489352, 0.86]]

#4loooopy=[[28.768632945162715, 77.11563521474226, 0.87], [28.766181755175356, 77.1174036663717, 0.84], [28.76666614865009, 77.11106759341759, 0.91], [28.768237243626746, 77.11602507606865, 0.92], [28.768345162200898, 77.11627130427482, 0.73], [28.76711580947471, 77.11078033184822, 0.97], [28.7661457823142, 77.11736262940019, 0.98], [28.76813786370445, 77.1115063401752, 0.79], [28.767652230068165, 77.11109596220277, 0.74], [28.768219257172984, 77.11631234230917, 0.82], [28.768596972293842, 77.11569677179378, 0.99], [28.76823724365767, 77.11561469572507, 0.94], [28.76601987730948, 77.11709588908548, 0.76], [28.768345162221, 77.11600455705148, 0.96], [28.766756080792128, 77.11131381761989, 0.73], [28.76838113508523, 77.11600455705148, 0.9], [28.76569612154836, 77.11687018574226, 0.74], [28.76603786372149, 77.11736262940019, 0.77], [28.76589397227536, 77.11721899999996, 0.98], [28.767993972273807, 77.11115751889862, 0.86]]

#3loooopy=[[28.768938918006867, 77.11657496185941, 0.83], [28.76786386372456, 77.11170059422388, 0.92], [28.76768399938944, 77.11188526382665, 0.95], [28.771584904432768, 77.11358003917407, 0.7], [28.76892093159333, 77.11632873301619, 0.78], [28.765324850190733, 77.11028618818531, 0.97], [28.768705094387773, 77.11659548092969, 0.99], [28.76478523006667, 77.116640481665, 0.71], [28.767773931563973, 77.11170059422388, 0.76], [28.76500109438167, 77.11069655502628, 0.96], [28.77174678230792, 77.11376471545735, 0.89], [28.767881850179876, 77.11139281155258, 0.89], [28.76494710797738, 77.11635322497553, 0.8], [28.765126999400305, 77.11077862839447, 0.86], [28.76886697225056, 77.11694430512425, 0.73], [28.765216904456064, 77.11639426164545, 0.9], [28.768687107947947, 77.11669807628104, 0.75], [28.771297121529674, 77.11343640206482, 0.99], [28.767935809476235, 77.11139281155258, 0.88], [28.7691007958743, 77.11686222884319, 0.89]]


#2loooopy=[[28.76574104014106, 77.11494818595281, 0.73], [28.770886782300153, 77.11192531068663, 0.76], [28.77050906726433, 77.11141232523872, 0.71], [28.766558945116405, 77.1115307791412, 0.96], [28.76773901298361, 77.11169244342281, 0.9], [28.77090476876937, 77.11143284465663, 0.84], [28.766433040142605, 77.11085366503062, 0.94], [28.7674872029355, 77.1116719245637, 0.73], [28.770759877284647, 77.1170951941773, 0.8], [28.765525202955633, 77.11494818595281, 0.78], [28.76548923004501, 77.1155637401539, 0.92], [28.77031121651722, 77.11133024756705, 0.94], [28.7679008908464, 77.11204126402775, 0.86], [28.770652958696537, 77.11174063592537, 0.81], [28.765938890854162, 77.11548166626041, 0.88], [28.770921755173717, 77.1170951941773, 0.77], [28.770455107957158, 77.11155596116413, 0.82], [28.770813836604166, 77.11678740291143, 0.84], [28.770293230069644, 77.11153544174621, 0.74], [28.7705800129619, 77.11711571359501, 0.7]]
#21loooopy=[[28.76739727031142, 77.11144621743176, 0.84], [28.766217202433324, 77.11101781411806, 0.9], [28.76739727007895, 77.1120617825683, 0.77], [28.766846727441234, 77.11132559294094, 0.94], [28.765435270350245, 77.11488663095705, 0.97], [28.765525202510847, 77.11488663095705, 0.87], [28.765794998760175, 77.11550218452149, 0.81], [28.766127270079004, 77.11153077882287, 0.83], [28.765705066754556, 77.11509181547854, 0.94], [28.766307134671404, 77.11081262823615, 0.71], [28.766064795203217, 77.11560477678223, 0.74], [28.76757713440014, 77.1120617825683, 0.86], [28.765705066870773, 77.1147840386963, 0.94], [28.766217202278344, 77.1114281858819, 0.98], [28.76766706663822, 77.11185659418946, 0.82], [28.76802679547433, 77.11134362324233, 0.73], [28.77023827004004, 77.11730038793006, 0.92], [28.770958727673552, 77.11108401508257, 0.89], [28.77095772751855, 77.1167874030175, 0.79], [28.76811672767367, 77.11124102905292, 0.8]]
#22loooopy=[[28.76684672759622, 77.1109152211771, 0.88], [28.770687931191773, 77.1163770150875, 0.8], [28.76621720239458, 77.11112040705902, 0.85], [28.767936863274993, 77.11144621743176, 0.71], [28.770149338228208, 77.11108401508257, 0.7], [28.767397270040203, 77.11216437675773, 0.81], [28.767846931075646, 77.11154881162119, 0.95], [28.765974863042622, 77.11560477678223, 0.95], [28.770148337918194, 77.11719779094754, 0.79], [28.767936863197495, 77.1116514058106, 0.96], [28.770868795357952, 77.1114944030165, 0.92], [28.766666863158804, 77.111223, 0.95], [28.766397066560785, 77.11153077882287, 0.83], [28.76684672751871, 77.11112040705902, 0.82], [28.77050806656057, 77.11719779094754, 0.94], [28.767487202472015, 77.11144621743176, 0.93], [28.767577134710105, 77.11124102905292, 0.86], [28.770778863003606, 77.1120073879339, 0.76], [28.765794998876398, 77.11519440773928, 0.91], [28.76684672763496, 77.11081262823615, 0.93]]
#31
loooopy=[[28.767234337879614, 77.11200837647061, 0.95], [28.76503706656088, 77.11094277481241, 0.72], [28.769190727479884, 77.11661600000002, 0.95], [28.771189202549362, 77.1130260108547, 0.71], [28.764857202510864, 77.11022463358348, 0.87], [28.764911134632722, 77.11635322529376, 0.79], [28.771818727634766, 77.11312860868375, 0.81], [28.767683998953796, 77.11129021764707, 0.84], [28.765306863158877, 77.11063500000002, 0.93], [28.76512699903139, 77.11012204197935, 0.87], [28.77172879551293, 77.1130260108547, 0.92], [28.771009337879406, 77.11394939131628, 0.76], [28.765090998760208, 77.11686618313746, 0.83], [28.768741066754412, 77.1164108095095, 0.75], [28.767683998798812, 77.11170059411766, 0.83], [28.76804372747994, 77.11159800000001, 0.7], [28.76883099879877, 77.11671859524529, 0.99], [28.764947134361545, 77.11104536641655, 0.88], [28.767683998876304, 77.11149540588237, 0.77], [28.76750413436139, 77.11200837647061, 0.97]]
#41loooopy=[[28.76786806652197, 77.11154737754809, 0.91], [28.768650930843126, 77.11635337991926, 0.75], [28.767223727402488, 77.11117018662311, 0.77], [28.768317727557413, 77.110931811226, 0.9], [28.7678680667157, 77.11103440561301, 0.85], [28.767868066676954, 77.11113700000003, 0.72], [28.766217727325078, 77.11762936929073, 0.88], [28.767508338073323, 77.11103440561301, 0.94], [28.768381134400084, 77.11625078493945, 0.97], [28.768560998682528, 77.11635337991926, 0.75], [28.766217727363824, 77.11752677696806, 0.8], [28.765678134516477, 77.11721900000005, 0.82], [28.76838113451633, 77.11594300000003, 0.78], [28.768111338228287, 77.11543002510092, 0.99], [28.768227795319337, 77.11113700000003, 0.74], [28.766684134438925, 77.11117018662311, 0.89], [28.765768066599588, 77.11742418464539, 0.73], [28.768201270117647, 77.11614818995965, 0.8], [28.766684134516417, 77.110965, 0.78], [28.768560998760023, 77.11614818995965, 0.88]]


#6loooopy=[[28.765432768771046, 77.11045033492171, 0.78], [28.764785230054297, 77.11680462834472, 0.96], [28.765324850164422, 77.11063500000014, 0.88], [28.769136768729258, 77.11698534326479, 0.88], [28.76509102652369, 77.11094277513085, 0.79], [28.76741420291539, 77.11178266960289, 0.86], [28.767486148636138, 77.11188526382665, 0.86], [28.764749257177684, 77.11696877502442, 0.97], [28.768543216498717, 77.11659548092969, 0.72], [28.76516294516125, 77.1163737433105, 0.81], [28.765126972301672, 77.1163121883056, 0.88], [28.767540107980413, 77.1112491796393, 0.8], [28.77138705369645, 77.11335432371668, 0.92], [28.765144985816956, 77.11098381181496, 0.73], [28.771746782351197, 77.1131901670204, 0.72], [28.769046836624316, 77.11624665673511, 0.9], [28.765306863709107, 77.11094277513085, 0.8], [28.765073040091572, 77.11094277513085, 0.74], [28.76525290443132, 77.11069655502628, 0.71], [28.771189202926138, 77.11358003917407, 0.91]]
#1loooopy=[[28.76405159075026, 77.11329134780544, 0.72], [28.768134510831977, 77.11341445650442, 0.95], [28.768602158091763, 77.11308616664049, 0.77], [28.771138244596663, 77.11870813056039, 0.71], [28.766533718653275, 77.10970067741864, 0.88], [28.770940394084693, 77.11550730438701, 0.92], [28.77067059760129, 77.11552782250351, 0.73], [28.765562451062106, 77.11310668475697, 0.75], [28.771497973809716, 77.11113694557336, 0.8], [28.765742314984234, 77.11840035881295, 0.92], [28.766371840824515, 77.10890047087528, 0.82], [28.766461772680415, 77.112942539825, 0.91], [28.770041072291605, 77.11798999648303, 0.8], [28.77246924104671, 77.11242958691261, 0.98], [28.769969126624886, 77.11716927182319, 0.76], [28.76487896623174, 77.11854398562842, 0.87], [28.766695596155646, 77.11483020654263, 0.94], [28.766893447141076, 77.11175248906824, 0.93], [28.768997859793167, 77.11050088396198, 0.9], [28.77225340406693, 77.10970067741864, 0.85], [28.765094803565653, 77.1165742464448, 0.88], [28.771875688324258, 77.11856450374492, 0.95], [28.766263922086463, 77.11082917382592, 0.78], [28.76550849146879, 77.11704616312421, 0.88], [28.764627156856452, 77.10959808683616, 0.74], [28.768386321207917, 77.10908513392376, 0.96], [28.765562450658468, 77.11846191316243, 0.73], [28.770346841960727, 77.11370171013536, 0.84], [28.767540958700415, 77.11171145283525, 0.71], [28.77086844849695, 77.11364015578587, 0.97]]
#byhandloooopy=[[28.763781793847798, 77.11887227549236, 0.97], [28.765148762948655, 77.11542523192102, 0.97], [28.766569691533, 77.10949549625367, 0.83], [28.76451923802555, 77.11275787677654, 0.77], [28.769807249187487, 77.11117798180635, 0.8], [28.769033832221353, 77.11628699281385, 0.71], [28.766299894596486, 77.11552782250351, 0.98], [28.771965620636482, 77.1165537283283, 0.86], [28.772001593813062, 77.11240906879613, 0.72], [28.769915167362598, 77.11671787326027, 0.79], [28.766084057721976, 77.11140368108781, 0.91], [28.768530212425183, 77.11226544198064, 0.86], [28.76826041576391, 77.11464554349419, 0.87], [28.767397067275922, 77.11128057238882, 0.89], [28.766965392503007, 77.1166152826778, 0.89], [28.763691862318232, 77.11050088396198, 0.94], [28.76509480342492, 77.11844139504593, 0.96], [28.770958380383764, 77.11727186240567, 0.91], [28.767900687517482, 77.1093929056712, 0.98], [28.764357360260213, 77.11111642745686, 0.99], [28.767918673583104, 77.11425569928075, 0.7], [28.764896952625204, 77.11905693854084, 0.73], [28.771497973699876, 77.11259373184457, 0.73], [28.769627384968402, 77.10982378611762, 0.98], [28.766803514994393, 77.11156782601977, 0.82], [28.76617398994594, 77.11056243831148, 0.76], [28.7641595092084, 77.1150764239406, 0.75], [28.765436545696996, 77.1176206703861, 0.89], [28.77013100481559, 77.11316823910646, 0.75], [28.771749783857725, 77.11115746368985, 0.97]]


while True:
    if timer == TMAX or distance(finalwaypoints[uav.id-1][0], finalwaypoints[uav.id-1][1], uav.get_pos()[0],uav.get_pos()[1])<5:
        #fix this
        print("ending mopso search...")
        print("...")
        break

    # for testing
    a1,a2,a3,a4=rectangle2(uav.get_pos(),uav.heading(),fol,fow)
    for i in loooopy:
        point = Point(i[0], i[1])
        polygon = Polygon([(a1[0],a1[1]), (a2[0],a2[1]), (a3[0],a3[1]), (a4[0],a4[1])])
        if polygon.contains(point):

            uav.update_PersonalHumans(i)

            detected_humans.append(i)

    data_to_send = {uav.id:{"GPS":uav.get_pos(), "Humans":detected_humans, "G":uav.get_gbest(), "P":uav.get_pbest(), "bestloc":uav.get_bestlocation(), "gbestloc":uav.get_gbestloc()}}
    # Code for sending the data
    for i in range(n):
        send_data(sending_sock[i], local_address, local_port + i, data_to_send)

    # Code for receiving the data
    for i in range(n):
        GlobalUavData[i+1] = recv_data(receiving_sock[i])

    # shared human list update
    shared_human_list = set()
    for i in GlobalUavData:
        for humans in GlobalUavData[i]["Humans"]:
            shared_human_list.add(humans)    # multiple detection

        shared_human_list = list(shared_human_list)

    # gbest and p best updation
    for humans in shared_human_list:
        nearestuav = uav.minimumdist(GlobalUavData)
        uav.update_pbest() 
        uav.updategbest(GlobalUavData, nearestuav)

        uav.payload_drop(GlobalUavData, humans)

    
    # check uav state
    if uav.get_state() == 0:
        uav.checkifpayload()
        vel = uav.velocity_post_drop()
        uav.update_vel(vel)
        print(vel)

    else:
        vel = uav.generate_mopso_velocity(timer, finalwaypoints, num, TMAX, wstart, wend)
        uav.update_vel(vel)
        print(vel)
        uav.change_gpbest(finalwaypoints, p1, p4)

    timer += 1
    time.sleep(1)

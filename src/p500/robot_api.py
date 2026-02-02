#!/usr/bin/python
# -*- coding: utf-8 -*-

import io
import sys
import json
import time
import socket
import requests  # pip3 install requests
import websocket  # pip3 install websocket-client
from typing import Any


host = "http://192.168.1.100:8080"
sess = requests.Session()

class Res:
    def __init__(self, b, r):
        # type:(bool, requests.Response)->None
        self.code = -1  # type:int
        self.data = None  # type: Any
        self.msg = str(r.reason)  # type:str
        if b:
            if "application/json" in r.headers.get("Content-Type", ""):
                d = r.json()
                self.code, self.data, self.msg = d["code"], d["data"], d["msg"]
            else:
                self.code = 0
                self.data = r._content
        self.__text = json.dumps(self, default=lambda o: o.__dict__ if hasattr(o, '__dict__') else '<not serializable>')

    def __bool__(self):
        # type:()->bool
        return self.code == 0

    __nonzero__ = __bool__

    def __str__(self):
        # type:()->str
        return self.__text


def api(method, api):
    def wrapper(fun):
        def wrapper(*args, **kwargs):
            try:
                u, param = fun(*args, **kwargs)
                url = '{}{}{}'.format(host, api, u)
                res = sess.request(method, url, **param)
                token = res.headers.get("Authorization", "")
                if len(token) != 0:
                    sess.headers.update({"Authorization": token})
                return Res(True, res) if res.ok else Res(False, res)
            except requests.exceptions.RequestException as e:
                res = requests.Response()
                res.reason = e.__str__() # type: ignore
                res.status_code = 0 # type: ignore
                return Res(False, res)
        return wrapper
    return wrapper


def udpDiscovery():
    # type:()->tuple[bool, dict[str, str]]
    ips = dict()  # type: dict[str, str]
    data = 'discovery'.encode('utf-8')
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.settimeout(2.0)
    try:
        for i in range(0, 5):
            udp.sendto(data, ('224.0.0.1', 9999))
        while True:
            recv = udp.recvfrom(1024)
            if not len(recv):
                break
            ips[str(recv[0])] = recv[1][0]
    finally:
        udp.close()
        return True if len(ips) else False, ips


def discovery(label):
    # type:(str)->str
    try:
        return socket.gethostbyname(label + ".local")
    except:
        return udpDiscovery()[1].get(label, "")


def setHost(ip, https=False):
    # type:(str, bool)->None
    global host
    host = '{}://{}:{}'.format("https" if https else "http",
                               ip, 443 if https else 8080)


@api("GET", "/api/devices")
def httpDiscovery():
    # type:()->Res
    return "", {}  # type: ignore


@api("POST", "/api/user/login")
def login(username, password):
    # type:(str, str)->Res
    return "", {'json':  # type: ignore
                {'username': username, 'password': password}}


@api("GET", "/api/roslaunch/mapping")
def mapping():
    # type:()->Res
    return "", {}  # type: ignore


@api("GET", "/api/roslaunch/navigation/")
def navigation(name):
    # type:(str)->Res
    return name, {}  # type: ignore


@api("GET", "/api/roslaunch/stop")
def stop():
    # type:()->Res
    return "", {}  # type: ignore


@api("GET", "/api/roslaunch/status")
def status():
    # type:()->Res
    return "", {}  # type: ignore


@api("GET", "/api/map/save/")
def saveMap(name):
    # type:(str)->Res
    return name, {}  # type: ignore


@api("GET", "/api/map/list")
def listMap():
    # type:()->Res
    return "", {}  # type: ignore


@api("GET", "/api/map/delete/")
def deleteMap(name):
    # type:(str)->Res
    return name, {}  # type: ignore


@api("GET", "/api/map/download/")
def downloadMap(name):
    # type:(str)->Res
    return name, {}  # type: ignore


@api("POST", "/api/map/upload/")
def uploadMap(file):
    # type:(io.IOBase)->Res
    return "", {'files': {'file': file}}  # type: ignore


@api("GET", "/api/param/get/")
def getParam(name):
    # type:(str)->Res
    return name, {}  # type: ignore


@api("GET", "/api/scheduler/release")
def release(name):
    # type:(str)->Res
    return "?map="+name, {}  # type: ignore


@api("GET", "/api/param/set/")
def setParam(name, val):
    # type:(str, object)->Res
    return "/".join([name, val]), {}  # type: ignore


@api("GET", "/api/task/run/")
def runTask(name):
    # type:(str)->Res
    return name, {}  # type: ignore


@api("POST", "/api/task/set")
def setTaskNow(name, actions, loop=1):
    # type:(str, list[dict[str, object]], int)->Res
    return "", {'json':  # type: ignore
                {'name': name, 'mode': 0, 'loop': loop, 'actions': actions}}


@api("POST", "/api/task/set")
def setTaskOnce(name, hour, minute, actions, loop=1):
    # type:(str, int|list[int], int|list[int], list[dict[str, object]], int)->Res
    return "", {'json': {'name': name, 'mode': 1, 'loop': loop, 'actions': actions,  # type: ignore
                         'hour': hour if type(hour) == list else [hour],
                         'minute': minute if type(minute) == str else [minute]}}


@api("POST", "/api/task/set")
def setTaskWeekly(name, hour, minute, actions, week=[], loop=1):
    # type:(str, int|list[int], int|list[int], list[dict[str, object]], int|list[int], int)->Res
    return "", {'json': {'name': name, 'mode': 2, 'loop': loop, 'actions': actions,  # type: ignore
                         'opt': week if type(week) == list else [week],
                         'hour': hour if type(hour) == list else [hour],
                         'minute': minute if type(minute) == str else [minute]}}


@api("POST", "/api/task/set")
def setTaskMonthly(name, hour, minute, actions, month=[], loop=1):
    # type:(str, int|list[int], int|list[int], list[dict[str, object]], int|list[int], int)->Res
    return "", {'json': {'name': name, 'mode': 3, 'loop': loop, 'actions': actions,  # type: ignore
                         'opt': month if type(month) == list else [month],
                         'hour': hour if type(hour) == list else [hour],
                         'minute': minute if type(minute) == str else [minute]}}


@api("GET", "/api/robot/status")
def robotStatus():
    # type:()->Res
    return "", {}  # type: ignore


@api("GET", "/api/robot/clear")
def clear():
    # type:()->Res
    return "", {}  # type: ignore


@api("POST", "/api/robot/pause")
def pause(data):
    # type:(bool)->Res
    return "", {'json': {'data': data}}  # type: ignore


@api("POST", "/api/robot/charge")
def charge(data):
    # type:(bool)->Res
    return "", {'json': {'data': data}}  # type: ignore


@api("GET", "/api/ptz/")
def setPtz(h_angle,v_angle,lift):
    # type:(str, object)->Res
    return "/".join([h_angle,v_angle,lift]), {}  # type: ignore

@api("GET", "/api/ptz")
def getPtz():
    # type:(str, object)->Res
    return "", {}  # type: ignore

def ws(onOpen=None, onClose=None, onMsg=None):
    url = "{}/api/ws?token={}".format(host.replace("http", "ws"),
                                      sess.headers.get("Authorization", ""))
    websocket.WebSocketApp(url, on_open=onOpen,
                           on_close=onClose, on_message=onMsg).run_forever()


def send(conn, dataType, data):
    # type:(websocket.WebSocketApp, int, object)->None
    conn.send(json.dumps({'type': dataType, 'data': data}))


################################################################################
#                                                                              #
#                               example                                        #
#                                                                              #
################################################################################


def example():
    ok, ips = udpDiscovery()
    print(ok, ips)
    if not ok:
        print("no device found")
        sys.exit(0)

    setHost(list(ips.values())[0])

    # res = httpDiscovery()
    # print(res)

    res = login('admin', 'admin')
    print(res)

    res = status()
    print(res)

    res = listMap()
    print(res)
    if not len(res.data):
        print("no map, mapping first")
        sys.exit(0)

    res = navigation(res.data[0]['name'])
    print(res)

    res = robotStatus()
    print(res)

    # res = pause(True)
    # print(res)

    # res = downloadMap("123.png")
    # print(res)

    # res = uploadMap(open("./222.png", 'rb'))
    # print(res)

    # res = getParam("max_vel")
    # print(res)

    def onOpen(conn):
        # type:(websocket.WebSocketApp)->None
        print("open conn")
        # send(conn, 3, {'data': True})

    def onMsg(conn, msg):
        # type:(websocket.WebSocketApp, str)->None
        print(msg)

    def onClose(conn):
        # type:(websocket.WebSocketApp)->None
        print("close conn")

    ws(onOpen, onClose, onMsg)


def initNavi(label, mapname, username="admin", passwd="admin"):
    # type:(str, str, str, str)->tuple[bool, str]
    if label.startswith("puwei_robot_"):
        print("discovery device with label: {}".format(label))
        addr = discovery(label)
        if addr == "":
            return False, "no device found"
        print("{}: {}".format(label, addr))
        setHost(addr)
    else:
        setHost(label)  # 如果已知设备ip，可以跳过上述设备发现流程

    res = login(username, passwd)
    if not res:
        return False, "login failed: " + res.msg

    res = status()
    if res and res.msg == mapname:
        return True, ""

    if res and res.data == 1:  # not in idle or navigation state
        stop()
        time.sleep(1)

    if not navigation(mapname):
        return False, "launch navigation failed: " + res.msg
    print("start navigation with map: {}".format(mapname))

    time.sleep(5)
    while True:
        time.sleep(1)
        res = robotStatus()
        print(u"robot status: [{} - {} {}], waiting for startup...".format(
            res.data["state"], ','.join(res.data["err"]), res.data["msg"]))
        if res and res.data["state"] not in [-1, 4, 5]:
            break

    return True, ""


# label为机器标签或者机器IP地址
def naviTest(label, mapname, task, username="admin", passwd="admin"):
    # type:(str, str, str, str, str)->None
    ok, msg = initNavi(label, mapname, username, passwd)
    if not ok:
        print(msg)
        return

    clear()  # 清除错误
    res = runTask(task)
    if not res:
        print(res.msg)
        return

    while True:
        time.sleep(1)
        res = robotStatus()
        print(u"robot status: [{} - {} {}], position: {}]".format(
            res.data["state"], ','.join(res.data["err"]), res.data["msg"],
            res.data["pose"]["position"]))
        if res and "3-2" in res.data["err_code"]:
            print("\n========block at station {}\n".format(res.data["station"]))
            release(mapname)
        if res and res.data["state"] == 0:
            break


if __name__ == "__main__":
    # example()

    # 使用时需修改对应参数: 机器标签/机器IP、地图名称、任务名称
    naviTest("192.168.1.100", "testmap", "taskname")
    # naviTest("puwei_robot_xxxx", "testmap", "taskname")

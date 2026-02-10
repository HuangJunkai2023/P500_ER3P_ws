"""拖动示教并实时打印末端位姿 xyzrpy"""

import threading
import time
import setup_path
import platform

if platform.system() == 'Windows':
    from Release.windows import xCoreSDK_python
elif platform.system() == 'Linux':
    from Release.linux import xCoreSDK_python
else:
    raise ImportError("Unsupported operating system")


def log_result(step: str, ec: dict) -> bool:
    value = ec.get("value", 0)
    msg = ec.get("message", "")
    ok = (value == 0) or (msg in ("", "操作成功完成"))
    if ok:
        print(f"{step}: {msg if msg else 'ok'}")
        return True
    print(f"{step}: {msg if msg else 'failed'}")
    return False


def drag_teach_print_pose(robot, ec, remote_ip: str):
    # 1) 连接
    robot.connectToRobot(remote_ip)
    print("connectToRobot: ok")

    # 2) 清除错误
    ec.clear()
    robot.clearServoAlarm(ec)
    log_result("clearServoAlarm", ec)
    ec.clear()
    robot.recoverState(1, ec)
    log_result("recoverState(1)", ec)

    # 3) 使能（上电）
    ec.clear()
    robot.setOperateMode(xCoreSDK_python.OperateMode.manual, ec)
    if not log_result("setOperateMode(manual)", ec):
        return
    ec.clear()
    robot.setPowerState(True, ec)
    if not log_result("setPowerState(True)", ec):
        return

    # SDK要求：开启拖动前需要手动模式下电
    ec.clear()
    robot.setPowerState(False, ec)
    if not log_result("setPowerState(False) before drag", ec):
        return

    # 4) 开启示教（拖动）
    ec.clear()
    robot.enableDrag(xCoreSDK_python.DragParameterSpace.cartesianSpace,
                     xCoreSDK_python.DragParameterType.freely, ec, True)
    if not log_result("enableDrag", ec):
        return

    running = {"value": True}

    def wait_exit():
        input("拖动中，按回车退出...\n")
        running["value"] = False

    thread = threading.Thread(target=wait_exit, daemon=True)
    thread.start()

    # 5) 持续打印末端位姿 xyzrpy（单位: m, rad）
    while running["value"]:
        ec.clear()
        cart = robot.cartPosture(xCoreSDK_python.CoordinateType.endInRef, ec)
        value = ec.get("value", 0)
        msg = ec.get("message", "")
        if value == 0:
            print("xyzrpy: "
                  f"{cart.trans[0]}, {cart.trans[1]}, {cart.trans[2]}, "
                  f"{cart.rpy[0]}, {cart.rpy[1]}, {cart.rpy[2]}")
        else:
            print(f"cartPosture failed: {msg if msg else value}")
            ec.clear()
        time.sleep(0.1)

    # 6) 退出拖动
    ec.clear()
    robot.disableDrag(ec)
    log_result("disableDrag", ec)

    # 7) 失能（下电）
    ec.clear()
    robot.setPowerState(False, ec)
    log_result("setPowerState(False)", ec)


if __name__ == "__main__":
    try:
        ip = "192.168.0.160"
        # ER3 Pro / ER7 Pro 机型请使用 xMateErProRobot
        robot = xCoreSDK_python.xMateErProRobot()
        ec = {}
        drag_teach_print_pose(robot, ec, ip)
    except Exception as e:
        print(f"An error occurred: {e}")

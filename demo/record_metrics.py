import csv
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped


rclpy.init()
node = rclpy.create_node("demo_metrics_recorder")
f = open("/home/nitin/twinguard_ws/src/TwinGuard/demo/demo_run_metrics.csv", "w", newline="")
writer = csv.writer(f)
writer.writerow(["t", "trust", "residual", "authority_scale", "fault_label", "supervisor_mode", "hold"])
start = time.time()
state = {"fault": "", "mode": "", "hold": ""}


def on_trust(msg):
    writer.writerow([
        round(time.time() - start, 2),
        msg.point.x,
        msg.point.y,
        msg.point.z,
        state["fault"],
        state["mode"],
        state["hold"],
    ])
    f.flush()


def on_integrity_diag(msg):
    if msg.status:
        state["fault"] = msg.status[0].message


def on_supervisor_diag(msg):
    if msg.status:
        state["mode"] = msg.status[0].message
        for kv in msg.status[0].values:
            if kv.key == "hold":
                state["hold"] = kv.value


node.create_subscription(PointStamped, "/twinguard/trust_state", on_trust, 10)
node.create_subscription(DiagnosticArray, "/twinguard/integrity_diagnostics", on_integrity_diag, 10)
node.create_subscription(DiagnosticArray, "/twinguard/supervisor_diagnostics", on_supervisor_diag, 10)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
f.close()

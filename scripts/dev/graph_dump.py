import rclpy, json
from rclpy.node import Node
rclpy.init()
n = Node("graph_dump")
import time; time.sleep(15.0)          # let discovery settle
nodes = [(name, ns) for name, ns in n.get_node_names_and_namespaces()]
topics = n.get_topic_names_and_types()
edges, pubs, subs = [], {}, {}
for t, _types in topics:
    try:
        for e in n.get_publishers_info_by_topic(t):
            pubs.setdefault(t, set()).add(e.node_name)
        for e in n.get_subscriptions_info_by_topic(t):
            subs.setdefault(t, set()).add(e.node_name)
    except Exception:
        pass
out = {
    "nodes": sorted({nm for nm, _ in nodes}),
    "pubs": {t: sorted(v) for t, v in pubs.items()},
    "subs": {t: sorted(v) for t, v in subs.items()},
}
print(json.dumps(out))
rclpy.try_shutdown()

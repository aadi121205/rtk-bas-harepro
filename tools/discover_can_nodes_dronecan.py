import dronecan, time

def main():
    node = dronecan.make_node('/dev/ttyACM0', node_id=127, bitrate=1000000, baudrate=115200, bus_number=1)
    nodes = {}
    def on_status(event):
        nid = event.transfer.source_node_id
        if nid not in nodes:
            print(f"Discovered node id={nid}")
        nodes[nid] = event.message
    node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_status)

    start = time.time()
    while time.time() - start < 15:
        try:
            node.spin(timeout=0.5)
        except Exception as e:
            print("spin err", e)

    print("total nodes:", len(nodes))
    for nid, msg in nodes.items():
        print(nid, msg)
    node.close()

if __name__ == '__main__':
    main()

# import socket
import json
import readline
import os


# SIM_VIEW_PIPE = "/tmp/sim_view_pipe"
SIM_CTL_PIPE = "/tmp/sim_ctl_pipe"
CTL_SIM_PIPE = "/tmp/ctl_sim_pipe"

fifo_out = os.open(CTL_SIM_PIPE, os.O_RDWR)
fifo_in = os.open(SIM_CTL_PIPE, os.O_RDWR)

while True:
    print("act > ", end="")
    act = input()
    if act == "quit":
        break

    print("data > ", end="")
    raw_data = input()

    try:
        data = json.loads(raw_data)
    except Exception as e:
        if raw_data:
            data = raw_data
        else:
            data = "empty"

    json_data = json.dumps({"act": act, "data": data})
    os.write(fifo_out, bytes(json_data + "\n", "utf-8"))
    print("Sent:     {}".format(json_data))

    received = str(os.read(fifo_in, 2048), "utf-8")
    print(f"Received: {received}")

    if received == "exit":
        break

os.close(fifo_in)
os.close(fifo_out)

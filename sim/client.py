# import socket
import json
import readline
import os


# SIM_VIEW_PIPE = "/tmp/sim_view_pipe"
AUX_CTL_PIPE = "/tmp/aux_ctl_pipe"
CTL_AUX_PIPE = "/tmp/ctl_aux_pipe"

fifo_out = os.open(AUX_CTL_PIPE, os.O_WRONLY)
fifo_in = os.open(CTL_AUX_PIPE, os.O_RDWR)

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

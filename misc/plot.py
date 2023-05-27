
import matplotlib.pyplot as plt
import json
from sys import argv
import numpy as np


def plot_data(msg):
    if msg.get("type") == "heat":
        data = np.array(msg.get("data"))
        c = plt.imshow(data.T, cmap="hot", interpolation="nearest")
        plt.colorbar(c)
        plt.show()


if __name__ == "__main__":
    if len(argv) != 2:
        print("Argument required")
        exit(1)

    json_str = None
    with open(argv[1], "r") as f:
        json_str = f.read()

    # print(f"json: {json_str}")
    json_data = json.loads(json_str)
    if json_data is None:
        print("Failed to parse json data")
        exit(1)

    plot_data(json_data)

    exit(0)

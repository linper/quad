
import matplotlib.pyplot as plt
import json
from sys import argv
import numpy as np


def plot_data(msg):
    for m in msg:
        if m.get("type") == "heat":
            tick = m.get("tick")
            data = np.array(m.get("data"))
            extents = (-data.shape[0] / 2 * tick, data.shape[0] /
                       2 * tick, -data.shape[1] / 2 * tick, data.shape[1] / 2 * tick)
            c = plt.imshow(data.T, cmap="hot",
                           interpolation="nearest", extent=extents)
            plt.colorbar(c)
        elif m.get("type") == "scatter":
            x_data = np.array(m.get("x"))
            y_data = np.array(m.get("y"))
            color = m.get("color")
            label = m.get("label")
            plt.scatter(x_data, y_data, color=color, label=label)
        elif m.get("type") == "plot":
            x_data = np.array(m.get("x"))
            y_data = np.array(m.get("y"))
            color = m.get("color")
            label = m.get("label")
            plt.plot(x_data, y_data, color=color, label=label)

    plt.legend()
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

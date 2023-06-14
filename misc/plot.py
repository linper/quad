
import matplotlib.pyplot as plt
import json
from sys import argv
import numpy as np


def get_xy_data(m):

    if m.get("data") is not None:
        data = np.array(m.get("data"))
        x_data = data[1]
        y_data = data[0]
    elif m.get("tdata") is not None:
        data = np.array(m.get("tdata"))
        tdata = np.transpose(data)
        x_data = tdata[1]
        y_data = tdata[0]
    else:
        x_data = np.array(m.get("x"))
        y_data = np.array(m.get("y"))

    return x_data, y_data


def plot_data(msg, datafile):
    plt.axis("equal")
    invert_y = True
    for m in msg:
        if m.get("type") == "heat":
            tick = m.get("tick")
            data = np.array(m.get("data"))
            extents = (-data.shape[0] / 2 * tick, data.shape[0] /
                       2 * tick, -data.shape[1] / 2 * tick, data.shape[1] / 2 * tick)
            c = plt.imshow(data.T, cmap="hot",
                           interpolation="nearest", extent=extents)
            plt.colorbar(c)
            invert_y = False
        elif m.get("type") == "scatter":
            x_data, y_data = get_xy_data(m)
            color = m.get("color")
            label = m.get("label")
            plt.scatter(x_data, y_data, color=color, label=label)
        elif m.get("type") == "plot":
            x_data, y_data = get_xy_data(m)
            color = m.get("color")
            label = m.get("label")
            marker = m.get("marker")
            linestyle = m.get("linestyle")
            plt.plot(x_data, y_data, color=color, label=label,
                     marker=marker, linestyle=linestyle)
        elif m.get("type") == "ring":
            x_data, y_data = get_xy_data(m)
            x_data = np.array(x_data.tolist() + [x_data[0]])
            y_data = np.array(y_data.tolist() + [y_data[0]])
            color = m.get("color")
            label = m.get("label")
            marker = m.get("marker")
            linestyle = m.get("linestyle")
            plt.plot(x_data, y_data, color=color, label=label,
                     marker=marker, linestyle=linestyle)

    if invert_y:
        plt.gca().invert_yaxis()
    # plt.gca().invert_yaxis()
    plt.legend()
    plt.savefig(datafile.replace(".json", ".png"))
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

    plot_data(json_data, argv[1])

    exit(0)

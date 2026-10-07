import matplotlib.pyplot as plt
import math
from utilities import FileReader
import argparse
import os


def plot_laser(filename):
    """
    Plot the first LaserScan message from a laser_content*.csv file.

    LaserScan CSV format:
        ranges, angle_increment, stamp

    The ranges field is stored as:
        array('f', [r1, r2, r3, ...])
    """

    # ---------------------------------------------------------
    # Read the first LaserScan message directly.
    # We do not use FileReader here because the ranges field
    # contains an entire array rather than a single float.
    # ---------------------------------------------------------
    with open(filename, "r") as f:
        f.readline()  # Skip header
        line = f.readline().strip()

    # ---------------------------------------------------------
    # Extract the range array
    # ---------------------------------------------------------
    start = line.find("[")
    end = line.find("]")

    if start == -1 or end == -1:
        raise ValueError(
            f"Could not find LaserScan range array in {filename}"
        )

    ranges_string = line[start + 1:end]

    # ---------------------------------------------------------
    # Extract angle_increment
    # Everything after the closing ']':
    #
    # ), angle_increment, timestamp
    # ---------------------------------------------------------
    remaining = line[end + 1:].strip()

    if remaining.startswith(","):
        remaining = remaining[1:].strip()

    parts = remaining.split(",")

    if len(parts) < 1:
        raise ValueError(
            f"Could not find angle_increment in {filename}"
        )

    angle_increment = float(parts[0].strip())

    # ---------------------------------------------------------
    # Convert range values to floats
    # ---------------------------------------------------------
    ranges = []

    for value in ranges_string.split(","):
        value = value.strip()

        try:
            ranges.append(float(value))
        except ValueError:
            # Treat anything that cannot be converted as invalid.
            ranges.append(float("nan"))

    # ---------------------------------------------------------
    # Convert polar coordinates -> Cartesian coordinates
    #
    # IMPORTANT:
    # Keep the original index i. If a range is inf/nan,
    # we skip plotting it but DO NOT shift the angle of
    # subsequent measurements.
    #
    # Assumes the first beam starts at angle 0 because
    # angle_min is not present in the logged CSV.
    # ---------------------------------------------------------
    x = []
    y = []

    for i, r in enumerate(ranges):

        # Ignore invalid measurements
        if math.isinf(r) or math.isnan(r):
            continue

        angle = i * angle_increment

        x.append(r * math.cos(angle))
        y.append(r * math.sin(angle))

    # ---------------------------------------------------------
    # Plot
    # ---------------------------------------------------------
    plt.figure()

    plt.scatter(x, y, s=5)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("First LaserScan")

    plt.axis("equal")
    plt.grid()

    plt.show()


def plot_errors(filename):
    """
    Plot regular sensor CSV files using FileReader.
    """

    headers, values = FileReader(filename).read_file()

    time_list = []
    first_stamp = values[0][-1]

    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        plt.plot(
            time_list,
            [lin[i] for lin in values],
            label=headers[i] + " linear"
        )

    plt.legend()
    plt.grid()
    plt.show()


def is_laser_file(filename):
    """
    Check whether this is a LaserScan CSV.

    All LaserScan files begin with:
        laser_content
    """

    basename = os.path.basename(filename)

    return basename.startswith("laser_content")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Plot logged sensor data."
    )

    parser.add_argument(
        "--files",
        nargs="+",
        required=True,
        help="List of files to process"
    )

    args = parser.parse_args()

    print("plotting the files", args.files)

    for filename in args.files:

        if is_laser_file(filename):
            print("Detected LaserScan file:", filename)
            plot_laser(filename)

        else:
            print("Detected regular sensor file:", filename)
            plot_errors(filename)
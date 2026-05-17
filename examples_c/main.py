import json
import re
import sys

from requests import request
import requests

# -----------------------------
# Regex patterns
# -----------------------------

ALIVE_RE = re.compile(
    r'^(RECEIVING|SENDING)\s+ALIVE_PING\s+(\d+)\s+([-\d.]+),([-\d.]+),([-\d.]+)\s+(\d+)$'
)

# Example:
# RECEIVING DRONE_MISSING SENDING DRONE_LASTSEEN 2 0 1 65926
MISSING_RE = re.compile(
    r'^RECEIVING\s+DRONE_MISSING\s+SENDING\s+DRONE_LASTSEEN\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)$'
)

# Example:
# RECEIVING DRONE_LASTSEEN 1 0 62454 67651
LASTSEEN_RE = re.compile(
    r'^RECEIVING\s+DRONE_LASTSEEN\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)$'
)


# -----------------------------
# Parsers
# -----------------------------

def parse_alive(line):
    match = ALIVE_RE.match(line)
    if not match:
        return None

    direction_raw, sender, x, y, z, timestamp = match.groups()

    return {
        "packet": "alive-ping",
        "direction": "receive" if direction_raw == "RECEIVING" else "send",
        "sender": int(sender),
        "position": {
            "x": float(x),
            "y": float(y),
            "z": float(z),
        },
        "time": int(timestamp),
    }


def parse_missing(line):
    match = MISSING_RE.match(line)
    if not match:
        return None

    sender, missing, last_seen, timestamp = match.groups()

    return {
        "packet": "device-missing",
        "direction": "send",
        "last-seen": None,
        "sender": int(sender),
        "missing": int(missing),
        "position": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "time": int(timestamp),
    }


def parse_lastseen(line):
    match = LASTSEEN_RE.match(line)
    if not match:
        return None

    sender, missing, last_seen, timestamp = match.groups()

    return {
        "packet": "device-missing",
        "direction": "receive",
        "last-seen": int(last_seen),
        "sender": int(sender),
        "missing": int(missing),
        "position": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "time": int(timestamp),
    }


def parse_line(line):
    line = line.strip()

    parsers = [
        parse_alive,
        parse_missing,
        parse_lastseen,
    ]

    for parser in parsers:
        result = parser(line)
        if result:
            return result

    return None


# -----------------------------
# Main
# -----------------------------


def main():
    uuid = 0
    if len(sys.argv) > 1:
        uuid = int(sys.argv[1])
    infile = sys.stdin

    for line in infile:
        parsed = parse_line(line)
        if parsed:
            print(json.dumps(parsed, indent=4))
            resp = requests.post(f"http://172.20.10.4:8000/data/{uuid}", json=parse_line(line), headers={"Content-Type": "application/json"})
            print(resp.status_code)



if __name__ == "__main__":
    main()
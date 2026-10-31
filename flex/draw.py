#!/usr/bin/env python3
"""Draws protocols for the flexibility trial from a public random value. See README.md in this folder."""
import argparse
import csv
import hashlib
import pathlib
import re

DECODERS = pathlib.Path(__file__).with_name("decoders.csv")


def read_pool(path):
    with open(path, newline="") as f:
        return [row["decoder"] for row in csv.DictReader(f) if row["status"] == "pool"]


def draw(pool, randomness, count):
    def rank(name):
        return hashlib.sha256(f"{randomness}:{name}".encode()).hexdigest()

    return sorted(pool, key=rank)[:count]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("randomness", help="the 64 hex digit randomness field of the drand round")
    parser.add_argument("--count", type=int, default=3, help="how many protocols to draw")
    args = parser.parse_args()

    randomness = args.randomness.lower()
    if not re.fullmatch(r"[0-9a-f]{64}", randomness):
        parser.error("randomness must be 64 hex digits")
    pool = read_pool(DECODERS)
    if not 1 <= args.count <= len(pool):
        parser.error(f"count must be from 1 to {len(pool)}")
    for name in draw(pool, randomness, args.count):
        print(name)


if __name__ == "__main__":
    main()

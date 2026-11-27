"""Common conversions at the encoder, wheel and lidar interfaces."""
import math


def travel_to_turns(distance_mm, diameter_mm):
    return distance_mm / (math.pi * diameter_mm)


def turns_to_counts(turns, counts_per_turn):
    return round(turns * counts_per_turn)


def counts_to_travel(counts, diameter_mm, counts_per_turn):
    return counts * math.pi * diameter_mm / counts_per_turn


degrees = math.degrees
radians = math.radians

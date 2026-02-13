# shelter.py
import random


def select_shelters(nodes, count):
    if not nodes:
        return set()
    return set(random.sample(nodes, min(count, len(nodes))))

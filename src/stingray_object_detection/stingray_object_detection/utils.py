#!/usr/bin/env python3


def get_objects_topic(camera_topic: str) -> str:
    return "%s/%s/objects" % (camera_topic, load_config()["nodes"]["object_detection"])

def get_debug_image_topic(camera_topic: str) -> str:
    return "%s/%s/image" % (camera_topic, load_config()["nodes"]["object_detection"])

#!/usr/bin/env python
# coding=UTF-8
import yaml

def load_config(config_path):
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)

#! /usr/bin/env python3

import json
import pandas as pd


data = pd.read_excel("turret-calibration-use.xlsx", sheet_name="CanCoder Pot calibration data")

print(data)


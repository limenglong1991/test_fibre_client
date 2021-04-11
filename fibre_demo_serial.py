#!/usr/bin/env python3
"""
Set the color on a fibre-enabled light controller.
"""
import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)) + "/fibre/python")
import fibre
  

# Parse arguments
parser = argparse.ArgumentParser(description='Set the color on a fibre-enabled light controller.')
parser.add_argument("-v", "--verbose", action="store_true",
                    help="print debug information")
'''
parser.add_argument("--host", metavar="HOSTNAME", action="store",
                    help="Specifies the host or IP address of the light controller.")
parser.add_argument("color", metavar="WWRRGGBB", type=str, nargs=1,
                    help="Specifies the color code.")
parser.add_argument("-t", "--time", metavar="SECONDS", type=float,
                    help="Fade duration. Defaults to 0.")
parser.add_argument("-l", "--limit-brightness", action="store_true",
                    help="don't increase brightness")
'''
parser.set_defaults(host="serial:COM4", time=0)
args = parser.parse_args()
'''
try:
  color = int(args.color[0], 16)
except ValueError:
  parser.print_usage(file=sys.stderr)
  sys.stderr.write("error: expected hexadecimal color code\n")
  sys.exit(1)
'''
# Connect to device
if (args.verbose):
  printer = print
else:
  printer = lambda x: None
fibre_serial = fibre.find_any(path=(args.host), timeout=5)

print(fibre_serial.fw_version_major)

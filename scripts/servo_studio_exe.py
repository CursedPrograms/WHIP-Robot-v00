"""servo_studio_exe.py - what builds/WHIP-Servo-Studio.exe runs.

The exe carries Python, pygame and the rest, and runs scripts/servo_studio.py from the repo beside it,
so it uses the same files, calibration and layout as the Python version and can't fall behind it.
"""
import os
import runpy
import sys

# Bundled for the scripts it runs: PyInstaller can't see those
import argparse, collections, http.server, json, math, random, re, requests, serial, shutil  # noqa: E401
import socketserver, subprocess, threading, time, urllib.parse, uuid, xml.sax.saxutils, zlib  # noqa: E401
import pygame  # noqa: F401

scripts = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(sys.executable))), "scripts")
if not os.path.exists(os.path.join(scripts, "servo_studio.py")):
    raise SystemExit("WHIP-Servo-Studio.exe stays in WHIP's builds folder, next to scripts")
sys.path.insert(0, scripts)
os.chdir(scripts)
runpy.run_path(os.path.join(scripts, "servo_studio.py"), run_name="__main__")

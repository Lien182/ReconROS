import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse

import tempfile
import subprocess

import os
from os import listdir
from os.path import isfile, join, isdir

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "deploy_copy"

def get_call(prj):
	return deploy_copy_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("deploy", description="""
		Copies the build outputs to the compute nodes
		""")
	parser.add_argument("-l", "--link", help="link sources instead of copying", action="store_true")
	parser.add_argument("-t", "--thread", help="export only single thread")
	parser.add_argument("hwdir", help="alternative export directory", nargs="?")
	return parser

def deploy_copy_cmd(args):
	print("Called deploy copy command")



